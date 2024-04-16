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

#include <ros/assert.h>
#include <yaml-cpp/yaml.h>

namespace cortex {
namespace util {

//! Extract the named YAML field or assert if the field doesn't exist.
YAML::Node GetFieldOrDie(const YAML::Node& node, const std::string& name) {
  auto field = node[name];
  ROS_ASSERT_MSG(field, "YAML field not found: %s", name.c_str());
  return field;
}

//! Extract a field of the specified type from the YAML node or assert if the field doesn't exist.
template <class T>
T GetOrDie(const YAML::Node& node, const std::string& name) {
  auto field = node[name];
  ROS_ASSERT_MSG(field, "Could not extract YAML field: %s", name.c_str());
  return field.as<T>();
}

}  // namespace util
}  // namespace cortex
