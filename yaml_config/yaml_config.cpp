#pragma once

#include "yaml_config.h"
#include <iostream>
#include <filesystem>

YAML::Node ReadConfigYamlFile();

// static std::string yaml_config_path =
// "/media/Projects/guangpo_lidar/src/guangpo_lidar/yaml_config/yaml_config.yaml";
static std::string yaml_config_path = []()->std::string{
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path yaml_path = current_dir / "yaml_config.yaml";
  return yaml_path;
}();

YAML::Node yaml_config = ReadConfigYamlFile();

YAML::Node ReadConfigYamlFile() {
  YAML::Node res;

  std::cout << __PRETTY_FUNCTION__ << ": " << std::endl;
  std::cout << "BEGIN READ FILE: " << yaml_config_path << std::endl;
  bool read_successful_flag = false;
  try {
    // Load the YAML file
    res = YAML::LoadFile(yaml_config_path);
    read_successful_flag = true;
  } catch (const YAML::Exception &e) {
    std::cerr << "Error while reading the YAML file: " << yaml_config_path
              << e.what() << std::endl;
    // gDebugError("[GXT] : Error while reading the YAML file:") << e.what();
  }
  if (!read_successful_flag) {
    // std::cerr << "backtrace:" << __PRETTY_FUNCTION__ << std::endl;
    // std::cerr << "backtrace:" << __PRETTY_FUNCTION__ << std::endl;
    // std::cerr << "backtrace:" << __PRETTY_FUNCTION__ << std::endl;
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::cerr << "Error while reading the YAML file!" << yaml_config_path
              << std::endl;
    std::terminate();
  }
  std::cout << "Read yaml config file successfully! " << yaml_config_path
            << std::endl;
  return res;
}

// __attribute((constructor)) inline bool
// GxtReadConfigYamlFile(const std::string &yaml_config_path) {
//   bool read_successful_flag = false;
//   try {
//     // Load the YAML file
//     YAML::Node config = YAML::LoadFile(yaml_config_path);
//
//     read_successful_flag = true;
//     // gDebugCol1("READ CONFIG FILE SUCCESSFULLY!");
//
//     auto quantize = config["quantize"];
//     ADDCONFIG(quantize, quantize_pose_quaternion_bit_width);
//     ADDCONFIG(quantize, quantize_pose_quaternion_il);
//     ADDCONFIG(quantize, quantize_pose_quaternion_bit_width);
//     ADDCONFIG(quantize, quantize_pose_quaternion_il);
//
//     ADDCONFIG(quantize, quantize_imu_speed_bit_width);
//     ADDCONFIG(quantize, quantize_imu_speed_il);
//     ADDCONFIG(quantize, quantize_imu_accbias_bit_width);
//     ADDCONFIG(quantize, quantize_imu_accbias_il);
//     ADDCONFIG(quantize, quantize_imu_gyrobias_bit_width);
//     ADDCONFIG(quantize, quantize_imu_gyrobias_il);
//
//     ADDCONFIG(quantize, quantize_inverse_depth_bit_width);
//     ADDCONFIG(quantize, quantize_inverse_depth_il);
//
//     ADDCONFIG(quantize, quantize_hessian_bit_width);
//     ADDCONFIG(quantize, quantize_hessian_il);
//     ADDCONFIG(quantize, quantize_b_bit_width);
//     ADDCONFIG(quantize, quantize_b_il);
//
//     auto flag = config["flag"];
//     ADDCONFIG(flag, use_gxt_backend);
//     ADDCONFIG(flag, enable_quantize);
//   } catch (const YAML::Exception &e) {
//     // std::cerr << "Error while reading the YAML file: " << e.what() <<
//     // std::endl;
//     gDebugError("[GXT] : Error while reading the YAML file:") << e.what();
//   }
//
//   if (!read_successful_flag) {
//     gDebugCol3("\n\n\n==========================================");
//     gDebugCol3("[GXT] : Error while reading the YAML file!");
//     gDebugCol3("[GXT] : Error while reading the YAML file!");
//     gDebugError("[GXT] : Error while reading the YAML file!");
//     std::terminate();
//   }
//
//   return 0;
// }
