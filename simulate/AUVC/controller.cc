#include "controller.h"
#include "mujoco/mjmodel.h"

#include <dlfcn.h>
#include <math.h>
#include <mujoco/mujoco.h>
#include <stdio.h>

static double Qorn[4] = {0}; // w,x,y,z
static double orn[3] = {0}; // r[forward axis],p[possibly side axis],y[vertical axis]
static double zOrn[3] = {0}; // Zero orientation

// a1,a2,a3,a4,a5,a6
const static float kp = 1.5;
const static float ki = 0.00005;
const static float kd = 0.00005;

static float error = 0;
static float last_error = 0;
static float intg = 0;
static float diff = 0;
static float prop = 0;
static float balance = 0;

mjtNum pid(mjtNum target, mjtNum currOrn);
void orn_Control(mjData* d, mjtNum target[3], mjtNum direction[3]);

void quaternion_to_euler(double x, double y, double z, double w, double *X,
                         double *Y, double *Z) {
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y * y);
  *X = atan2(t0, t1) * (180.0 / M_PI);

  double t2 = +2.0 * (w * y - z * x);
  if (t2 > 1)
    t2 = 1;
  else
    t2 = t2;
  if (t2 < -1)
    t2 = -1;
  else
    t2 = t2;
  *Y = asin(t2) * (180.0 / M_PI);

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  *Z = atan2(t3, t4) * (180.0 / M_PI);
}

extern "C" void controllerTestFunc(void) { printf("Hi from Plugin!\n"); }

extern "C" int controllerInitPlug(mjModel *m, mjData *d) {
  printf("Init Plug!\n");
  // Store Zero State data
  for(int i=0; i<4; i++){
    Qorn[i] = d->sensordata[i];
  }
  quaternion_to_euler(Qorn[1], Qorn[2], Qorn[3], Qorn[0], &zOrn[0], &zOrn[1], &zOrn[2]);
  printf("zOrn: [%f, %f, %f]", zOrn[0],zOrn[1],zOrn[2]);

  // Set this as the current baseline!
  // to set new Orientation, we simply say target[yaw] = 10;
  // i.e move yaw to zOrn[yaw] + currOrn[yaw] +  10:


  return 0;
}

extern "C" bool controllerUpdatePlug(mjModel *m, mjData *d) {
  // d->ctrl[0]= -1000.0;
  // d->ctrl[1]= 1000.0;
  // d->ctrl[2]= -1000.0;
  // d->ctrl[3]= 1000.0;
  // d->ctrl[4]=-1.0;
  // d->ctrl[5]=-1.0;
  return true;
  // int idx = mj_name2id(m, mjOBJ_ACTUATOR , "a6");

  // Capture Orientation
  //   [Px,Py,Pz, Ow,Ox,Oy,Oz]
  // Orientation: ^_________^
  for(int i=0; i<4; i++){
    Qorn[i] = d->sensordata[i];
  }

  quaternion_to_euler(Qorn[1], Qorn[2], Qorn[3], Qorn[0], &orn[0], &orn[1], &orn[2]);
  // printf("[0]: %0.3f | [1]: %0.3f | [2]: %0.3f\n", orn[0], orn[1], orn[2]);

  // Target Orientation is (90,0,0)
  double target[3] = {0, 0, 10};
  double direction[3] = {-1, 0, 1};
  orn_Control(d, target, direction);
  return true; // This line must be here
}

mjtNum pid(mjtNum target, mjtNum currOrn) {
    error = target - currOrn;
    prop = error;
    intg = error + intg;
    diff = error - last_error;
    balance = (kp * prop) + (ki * intg) + (kd * diff);
    last_error = error;
    return balance;
}

void orn_Control(mjData* d, mjtNum target[3], mjtNum direction[3]){
  int isYawCCW = direction[2];
  double yawError = target[2] + zOrn[2] - orn[2];
  // TODO: Controller Logic
  if( -170 >= target[2]  && target[2] >= -180){target[2] = - target[2];}
  if (yawError < 5) {
        // printf("c1\n");
        d->ctrl[0] = isYawCCW * -pid(target[2],orn[2]);
        d->ctrl[3] = isYawCCW * -pid(target[2],orn[2]);
        d->ctrl[1] = isYawCCW * pid(target[2],orn[2]);
        d->ctrl[2] = isYawCCW * pid(target[2],orn[2]);
  } else if (yawError > 5) {
        // printf("c2\n");
        d->ctrl[0] = isYawCCW * pid(target[2],orn[2]);
        d->ctrl[3] = isYawCCW * pid(target[2],orn[2]);
        d->ctrl[1] = isYawCCW * -pid(target[2],orn[2]);
        d->ctrl[2] = isYawCCW * -pid(target[2],orn[2]);
  } else{  // ccw
        // printf("ccw\n");
        d->ctrl[0] = isYawCCW*pid(target[2],orn[2]);
        d->ctrl[3] = isYawCCW*pid(target[2],orn[2]);
        d->ctrl[1] = isYawCCW*-pid(target[2],orn[2]);
        d->ctrl[2] = isYawCCW*-pid(target[2],orn[2]);
  }

  int isRollCCW = direction[0];
  double rollError = target[0]  + zOrn[0] - orn[0];

  double istargetNeg = (target[0]+0.0001)/(abs(target[0])+0.0001);
  if (rollError < istargetNeg * 5) {
        // printf("c1\n");
        d->ctrl[4] = isRollCCW * -pid(target[0],orn[0]);
        d->ctrl[5] = isRollCCW * pid(target[0],orn[0]);
  } else{  // ccw
        // printf("ccw\n");
        d->ctrl[4] = isRollCCW * pid(target[0],orn[0]);
        d->ctrl[5] = isRollCCW * -pid(target[0],orn[0]);
  }
}

// t0 = +2.0 * (w * x + y * z)
// t1 = +1.0 - 2.0 * (x * x + y * y)
// X = math.degrees(math.atan2(t0, t1))
//
// t2 = +2.0 * (w * y - z * x)
// t2 = +1.0 if t2 > +1.0 else t2
// t2 = -1.0 if t2 < -1.0 else t2
// Y = math.degrees(math.asin(t2))
//
// t3 = +2.0 * (w * z + x * y)
// t4 = +1.0 - 2.0 * (y * y + z * z)
// Z = math.degrees(math.atan2(t3, t4))
