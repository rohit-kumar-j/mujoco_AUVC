#include "mocap.h"
#include <compare>
#include <fstream>
#include <iostream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdlib.h>
#include <utils.h>
#include <vector>

// ----------------- Accessor Namespace -----------------
namespace Mocap {
bool _enable_log = false;
std::string _logging_prefix = "MOCAP ";
std::string mocap_name = "__uninitalized__";
std::string mocap_file_path = "__uninitalized__";
int nframes{};
int data_width{};
using MocapData = std::vector<std::vector<double>>;
MocapData mocap_data{{}}; // empty initliazer

// ------------ Private Functions in Mocap : Only in this file -------------
void _log(std::string msg) {
  if (_enable_log) {
    Utils ::_log(_logging_prefix, msg);
  }
}

void _throw_error_msg_and_exit(std::string e, bool exit_program) {
  Utils::_throw_error_msg_and_exit(_logging_prefix, e, exit_program);
}

void _read_json_from_file(const std::string &mocapName,
                          const std::string &filename,
                          std::vector<std::vector<double>> &data);

} // namespace Mocap

// --------------- Public Functions ---------------

bool Mocap::initialize(const std::string &mocapName,
                       const std::string &mocap_file_path,
                       std::optional<bool> enable_log) {
  Mocap::_enable_log = Utils::get_opt_boolean(enable_log);
  bool fs = std::filesystem::exists(mocap_file_path);
  if (!fs) {
    Mocap::_throw_error_msg_and_exit(
        "File: " + mocap_file_path + " does not exist!", true);
    return false;
  } else {
    Mocap::mocap_file_path = mocap_file_path;
    Mocap::_log("File found: " + mocap_file_path);
    Mocap::_read_json_from_file(mocapName, mocap_file_path, Mocap::mocap_data);
    Mocap::mocap_name = mocapName;
    return true;
  }
}

void Mocap::_print_metadata() {
  if (Mocap::mocap_name == "__uninitalized__") {
    Mocap::_throw_error_msg_and_exit(
        "mocap name is nil! Make sure to call Mocap::initialize()", true);
  }

  Mocap::_log("------- Mocap Metadata -------");
  Mocap::_log("Mocap ID: \"" + Mocap::mocap_name + "\"");
  Mocap::_log("Mocap File Path: \"" + Mocap::mocap_file_path + "\"");
  Mocap::_log("Number of frames in selected mocap: " +
              std::to_string(Mocap::nframes));
}

std::string Mocap::get_currently_selected_mocap_name() {
  if (Mocap::mocap_name == "__uninitalized__") {
    Mocap::_throw_error_msg_and_exit(
        "mocap name is nil! Make sure to call Mocap::initialize()", true);
  }
  return Mocap::mocap_name;
}

std::string Mocap::get_currently_selected_mocap_file_path() {
  if (Mocap::mocap_file_path == "__uninitalized__") {
    Mocap::_throw_error_msg_and_exit(
        "mocap_file_path is nil! Make sure to call Mocap::initialize()", true);
  }
  return Mocap::mocap_file_path;
}

// Returns the number of frames if in range, else -1
int Mocap::get_num_frames() {
  if (Mocap::nframes == 0) {
    Mocap::_log("Frames not found! make sure you call Mocap::initialize() "
                "before making calling other functions in Mocap:: namespace");
    return -1;
  } else {
    return Mocap::nframes;
  }
}

const std::vector<double> &Mocap::get_nth_frame(int n) {
  if (n >= Mocap::nframes) {
    Mocap::_throw_error_msg_and_exit(
        "Frame: " + std::to_string(n) +
            " is not found, there are only from: [0-" +
            std::to_string(Mocap::nframes),
        true);
  }
  return Mocap::mocap_data.at(n);
}

const std::vector<std::vector<double>> &Mocap::get_currently_selected_mocap() {
  if (Mocap::mocap_name == "__uninitalized__") {
    Mocap::_throw_error_msg_and_exit(
        "mocap name is nil! Make sure to call Mocap::initialize()", true);
  } else {
    return Mocap::mocap_data;
  }
  std::exit(EXIT_FAILURE);
}

// --------------- Private Functions ---------------

void Mocap::_read_json_from_file(const std::string &mocapName,
                                 const std::string &filePath,
                                 std::vector<std::vector<double>> &data) {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    Mocap::_throw_error_msg_and_exit("Failed to open JSON file: " + filePath,
                                     true);
    return;
  }

  nlohmann::json json_data;
  file >> json_data;

  // Read nrows and ncols
  int nrows = json_data["nrows"];
  int ncols = json_data["ncols"];
  std::string nameInJson = json_data["mocap_name"];
  Mocap::nframes = nrows;
  Mocap::data_width = ncols;
  Mocap::verify_mocap_name_is_same_as_file_name(mocapName, nameInJson);

  // Since we do this only once, we use try catch
  try {
    // Read mocap_data
    if (json_data["mocap_data"].is_array()) {
      for (const auto &row : json_data["mocap_data"]) {
        if (row.is_array()) {
          std::vector<double> rowData;
          for (const auto &element : row) {
            if (element.is_number()) {
              rowData.push_back(element.get<double>());
            }
          }
          data.push_back(rowData);
        }
      }
    }
  } catch (const std::exception &e) {
    std::cout << e.what();
  }
}

bool Mocap::verify_mocap_name_is_same_as_file_name(
    const std::string &mocapName, const std::string &nameInFile) {
  if (mocapName == nameInFile) {
    _log("Requested name: " + mocapName +
         " is the same as name in JSON file: " + nameInFile);
    return true;
  } else {
    Mocap::_throw_error_msg_and_exit(
        "Requested name: " + mocapName +
            " is the NOT same as name JSON file: " + nameInFile,
        true);
    return false;
  }
}
