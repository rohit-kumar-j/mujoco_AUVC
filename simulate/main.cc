// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <dlfcn.h> // AUVC Added this

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "mujoco/mjvisualize.h"
#include "simulate.h"
#include "array_safety.h"
#include "AUVC/Tags/tags.h"

#include "AUVC/Tags/AR_SRC/TagDetector.h"
#include "AUVC/Tags/AR_SRC/Tag36h11.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

#ifndef VIEWPORT_HEIGHT
#define VIEWPORT_HEIGHT 1080
#endif // VIEWPORT_HEIGHT
#ifndef VIEWPORT_WIDTH
#define VIEWPORT_WIDTH 1080
#endif // VIEWPORT_WIDTH


static bool initialize_controller_reload;

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

/*** AUVC ***/
#include <linux/videodev2.h>
static auvcData* avData; // helper struct for auvc Data (except cv::Mat)
static unsigned char* internal_color_buffer;

// Camera Thead


std::condition_variable bufferCV;
std::atomic<bool> bufferReady;
std::atomic<bool> bufferProcessed;

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

using Seconds = std::chrono::duration<double>;


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  // if no error and load took more than 1/4 seconds, report load time
  if (!loadError[0] && load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // do-while loop
      if(!initialize_controller_reload) {
        initialize_controller_reload = true;;
        plug_controller_init(m,d);
        // Code to be executed
        std::cout << "This will run only once." << std::endl;
        // Modify the condition to ensure the loop runs only once
      } while (initialize_controller_reload < 1); // Condition to check

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            // plug_physics_update(m,d);
            plug_controller_update(m,d);
            mj_step(m, d);
            stepped = true;
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              // plug_physics_update(m,d);
              plug_controller_update(m,d);
              mj_step(m, d);
              stepped = true;

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ Camera Thread --------------------------------------------------

void CameraThread(mj::Simulate* sim, auvcData* avData, unsigned char* internal_color_buffer){ for(int i =0; i<10000; i++){

    /* { // Init test
        std::string video_str = "/dev/video0";
        int deviceId = 0;
        video_str[10] = '0' + deviceId ;
        int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);
        int m_exposure(-1);
        int m_gain(-1);
        int m_brightness(-1);
        if (m_exposure >= 0) {
            // not sure why, but v4l2_set_control() does not work for
            // V4L2_CID_EXPOSURE_AUTO...
            struct v4l2_control c;
            c.id = V4L2_CID_EXPOSURE_AUTO;
            c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
            if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
                printf("Failed to set... %s\n", strerror(errno));
            }
            printf("exposure: %d\n",m_exposure);
            v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
        }
        if (m_gain >= 0) {
            printf("gain: %d\n", m_gain);;
            v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
        }
        if (m_brightness >= 0) {
            printf("brightness: %d\n", m_brightness);
            v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
        }
        v4l2_close(device);

        // find and open a USB camera (built in laptop camera, web cam etc)
        auto m_cap = cv::VideoCapture(deviceId);
        if(!m_cap.isOpened()) {
            printf("ERROR: Can't find video device %d\n",deviceId);
            exit(1);
        }
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // m_width
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800); // m_height
        printf("Camera successfully opened (ignore error messages above...)");
        printf("Actual resolution: %f x %f\n", m_cap.get(cv::CAP_PROP_FRAME_WIDTH), m_cap.get(cv::CAP_PROP_FRAME_HEIGHT) );

        cv::Mat image;
        cv::Mat image_gray;

        int frame = 0;
        double last_t = tic();
        while (true) {

            // capture frame
            m_cap >> image;

            // processImage(image, image_gray);
            cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
            // cv::imshow("Test Window", image); // OpenCV call

            // print out the frame rate at which image frames are being processed
            frame++;
            if (frame % 10 == 0) {

                double t = tic();
                printf("fps %f\n", 10./(t-last_t));
                last_t = t;
            }

            // exit if any key is pressed
            if (cv::waitKey(1) >= 0) break;
        }
    } */

    cv::Mat image(1080, 1080, CV_8UC3);
    cv::Mat image_gray(1080, 1080, CV_8UC3);
    cv::Mat flipped(1080, 1080, CV_8UC3);

    while(!sim->exitrequest.load()) {
      if(sim->camSync.load() == 1) {
        printf("Not Locked yet\n");
        {
          if (avData->color_buffer == NULL) {
            printf("Invalid buffer\n");
            return;
          }

          const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
          // cv::Mat image(1080, 1080, CV_8UC3, avData.color_buffer);
          // cv::Mat image_gray(1080, 1080, CV_8UC3, avData.color_buffer);
          // cv::Mat flipped(1080, 1080, CV_8UC3, avData.color_buffer);
          float pts[8];
          // Copy data using a loop
          // for (int i = 0; i < VIEWPORT_HEIGHT * VIEWPORT_WIDTH * 3; i++) {
          //   internal_color_buffer[i] = avData->color_buffer[i];
          // }
          //
          image.data = internal_color_buffer;
          int npts = auvc::processImage(image, image_gray, flipped, pts, NULL);

          // Simulate camera capturing data
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));

          // Write data
          avData->n_ar_tags = npts;
          for (int i = 0; i < avData->n_ar_tags; ++i) {
            avData->artag_corners[i] = pts[i];  // Dummy data
          }
          // Notify the camera thread that the buffer has been processed
          sim->camSync.store(0);
          sim->cond_camSync.notify_all();
          // sim->cond_camSync.wait(lock, [&]() { return sim->camSync == 1; });
        }
        printf("UnLocked\n");
      }
      // else{
      //   // printf("buffer not ready!\n");
      // }
    }
  }
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  initialize_controller_reload = 0;

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  if (!sim->reloadControllerLibPlug(m, d, AUVC_DEBUG, false)){
        printf("ERROR: Build the Controller first before launching the simualte executable: \"cmake --build <path-to-build-dir> --target Controller\" \n");
    } // Load controller
  if (!sim->reloadPhysicsLibPlug(m, d, AUVC_DEBUG, false)){
        printf("ERROR: Build the RoverPhysics first before launching the simualte executable: \"cmake --build <path-to-build-dir> --target RoverPhysics\" \n");
    } // Load controller

  const char* filename = "./model/flying-dog/flying-dog.xml";
  // const char* filename = "/home/rohit/Github/mujoco_AUVC/model/sub/sub.xml";
  if (argc >  1) {
    filename = argv[1];
  }

  // TODO: Video and IMU Stream
  // const char* imu_str = "/dev/ttyUSB0";

  // TODO: Allocate before camera thread

  /*** AUVC ***/
  avData = (auvcData*)malloc(sizeof(auvcData));
  avData->flg_render_ar_outlines = 1;
  avData->flg_render_lanes = 0;
  avData->n_ar_tags = 0;
  avData->n_lanes = 0;
  for(int i = 0; i< 4* auvcMaxArTags ;i++)
    avData->artag_corners[i] = 0;
  for(int i = 0; i< 4* auvcMaxArTags ;i++)
    avData->lane_corners[i] = 0;
  avData->ar_tag_rgba[0] = 1;
  avData->ar_tag_rgba[1] = 1;
  avData->ar_tag_rgba[2] = 1;
  avData->ar_tag_rgba[3] = 1;
  avData->lane_rgba[0] = 1;
  avData->lane_rgba[1] = 1;
  avData->lane_rgba[2] = 1;
  avData->lane_rgba[3] = 1;

  avData->color_buffer = (unsigned char*) malloc(VIEWPORT_HEIGHT * VIEWPORT_WIDTH * 3 * sizeof(unsigned char));
  internal_color_buffer = (unsigned char*) malloc(VIEWPORT_HEIGHT * VIEWPORT_WIDTH * 3 * sizeof(unsigned char));
  // Initialize avData.color_buffer with some data
  for (int i = 0; i < VIEWPORT_HEIGHT * VIEWPORT_WIDTH * 3; i++) {
    avData->color_buffer[i] = (unsigned char)(i % 256); // Example data
    internal_color_buffer[i] = (unsigned char)(i % 256); // Example data
  }

  // start camera thread
  // cvData.image       = cv::Mat(1080,1080, CV_8UC3);
  // cvData.flipped     = cv::Mat(1080,1080, CV_8UC3);
  // cvData.image_gray  = cv::Mat(1080,1080, CV_8UC1);
  // cvData.image = cv::Mat(1080,1080, CV_8UC3, avData.color_buffer);
  // cvData.image = cv::Mat(1080,1080, CV_8UC3, avData.color_buffer);
  // cv::Mat image(1080, 1080, CV_8UC3, &avData->color_buffer);
  // cv::Mat image_gray(1080, 1080, CV_8UC3, &avData->color_buffer);
  // cv::Mat flipped(1080, 1080, CV_8UC3, &avData->color_buffer);

  // std::thread camerathreadhandle(&CameraThread,sim.get(), avData, internal_color_buffer);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop(avData);
  physicsthreadhandle.join();
  // camerathreadhandle.join();

  if(libcontroller != NULL) {dlclose(libcontroller);}
  if(libphysics != NULL) {dlclose(libphysics);}

  free(avData->color_buffer);
  free(internal_color_buffer);

  return 0;
}
