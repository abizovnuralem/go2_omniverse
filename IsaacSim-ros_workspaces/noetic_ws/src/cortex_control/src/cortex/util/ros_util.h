/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

#include <iostream>
#include <sstream>

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <yaml-cpp/yaml.h>

namespace cortex {
namespace util {

//------------------------------------------------------------------------------
// Parameter helpers
//------------------------------------------------------------------------------

/*!\briefGeneric more convenient ROS parameter retrieval method that explicitly
 * returns the parameter value.
 *
 * Call as:
 *
 *   auto value = GetParam("/robot/step_size", .5);
 *   auto str_value = GetParam("/robot/controller_name", "lqr_controller");
 *
 * Infers the type by the type of the default value passed in.
 *
 * TODO: Figure out a way to get this to work with passing in const char*
 * string literals.
 */
template <class value_t>
value_t GetParam(const std::string& param_name, const value_t& default_value) {
  value_t param_value;
  ros::param::param(param_name, param_value, default_value);
  return param_value;
}

/*!\brief Call as: auto value = GetParam<double>("/robot/step_size"); Need to
 * specific supply the template argument for the parameter type.
 */
template <class value_t>
value_t GetParam(const std::string& param_name) {
  value_t param_value;
  ros::param::get(param_name, param_value);
  return param_value;
}

/*!\brief Get all parameters under a particular namespace.
 */
std::vector<std::string> GetNsParams(const std::string& ns);

/*!\brief Returns all of the names and corresponding tags under the given
 * namespace.
 *
 * Returns a vector of pairs with the first element being a name and the second
 * being a vector of strings for the tags:
 *
 *   /ns/first/1
 *   /ns/first/2
 *   /ns/second
 *   /ns/third/1
 *   /ns/third/2
 *   /ns/third/3
 *
 * Corresponding return structure:
 *
 *   { "first", {"1", "2"},
 *     "second", {},
 *     "third", {"1", "2", "3"} }
 *
 */
void GetNsElements(const std::string& ns, std::map<std::string, std::set<std::string>>& elements);


//------------------------------------------------------------------------------
// Subscription helpers
//------------------------------------------------------------------------------

/*!\brief Wait until connections to the publisher stabilize.
 *
 * Checks at a rate of rate_hz, and requires that the number of subscribers
 * doesn't change for stable_time seconds before considering the connection to
 * be stable and returning.
 */
void WaitForConnections(const ros::Publisher& pub, double stable_time = .2, double rate_hz = 30.);


//------------------------------------------------------------------------------
// Package helpers
//------------------------------------------------------------------------------

/*!\brief Converts a ROS package relative path into a full path.
 *
 * The ROS package relative path should take the form:
 * package://<pkg_name>/<rel_path>
 *
 * Returns <global_path_to_pkg>/<rel_path>
 *
 * For legacy reasons, currently the package:// prefix can be left off, but
 * that functionality is nonstandard with ROS and now deprecated. In the near
 * future, we'll require these strings to be prefixed with package://.
 */
std::string ExpandRosPkgRelPath(const std::string& pkg_relative_path);

}  // namespace util
}  // namespace cortex
