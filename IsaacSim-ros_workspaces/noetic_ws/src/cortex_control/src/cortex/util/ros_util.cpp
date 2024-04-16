/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/util/ros_util.h"

#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include "cortex/util/string.h"

namespace cortex {
namespace util {

void WaitForConnections(const ros::Publisher& pub, double stable_time, double rate_hz) {
  std::cout << "Waiting for connections" << std::flush;

  auto rate = ros::Rate(rate_hz);

  auto last_change_time = ros::Time::now();
  auto num_con = pub.getNumSubscribers();

  while (ros::ok()) {
    std::cout << '.' << std::flush;
    auto curr_time = ros::Time::now();

    auto latest_num_con = pub.getNumSubscribers();
    auto elapse_sec = (curr_time - last_change_time).toSec();
    if (latest_num_con != num_con) {
      num_con = latest_num_con;
      std::cout << num_con << std::flush;
      last_change_time = curr_time;
    } else if (latest_num_con > 0 && latest_num_con == num_con && elapse_sec >= stable_time) {
      std::cout << "<stable>" << std::endl;
      break;
    }
    rate.sleep();
  }
}

std::string ExpandRosPkgRelPathRaw(const std::string& pkg_relative_path) {
  // Parse out the json config file.
  char delim = '/';
  std::vector<std::string> tokens = Split(pkg_relative_path, delim);
  if (tokens.size() == 0) {
    return "";
  } else if (tokens.size() < 2) {
    return tokens.front();
  }
  auto pkg_name = tokens.front();
  auto rel_path = Join(tokens, delim, 1);  // Join all but first.

  auto package_path = ros::package::getPath(pkg_name);
  auto full_path = package_path + delim + rel_path;
  return full_path;
}

std::string ExpandRosPkgRelPath(const std::string& pkg_relative_path) {
  std::string expected_prefix = "package://";
  if (pkg_relative_path.find(expected_prefix) == 0) {
    return ExpandRosPkgRelPathRaw(pkg_relative_path.substr(expected_prefix.size()));
  } else {
    // The string doesn't start with the expected prefix, but we're still
    // supporting that for the time being. WARNING -- this functionality
    // is DEPRECATED; we'll require the package:// prefix soon.
    ROS_WARN_STREAM(
        "Package expansion without the 'package://' prefix is DEPRECATED: " << pkg_relative_path);
    return ExpandRosPkgRelPathRaw(pkg_relative_path);
  }
}

}  // namespace util
}  // namespace cortex
