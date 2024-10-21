#include <iostream>
#include <optional>
#include <string>
#include <vector>

#ifndef MOCAP_H
#define MOCAP_H
// Choose which mocap you want

namespace Mocap {
bool initialize(const std::string &mocapName,
                const std::string &mocap_file_name,
                std::optional<bool> enable_log = std::nullopt);
void _print_metadata();
std::string get_currently_selected_mocap_name();
std::string get_currently_selected_mocap_file_path();
bool verify_mocap_name_is_same_as_file_name(const std::string &mocapName,
                                            const std::string &nameInFile);
int get_num_frames();
const std::vector<double> &get_nth_frame(int n);
const std::vector<std::vector<double>> &get_currently_selected_mocap();
} // namespace Mocap

#endif /* MOCAP_H */
