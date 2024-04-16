/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

//! @file
//! @brief A simple and general joint state listener to collect the latest information
//! about the robot's state.

#pragma once

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "cortex/math/state.h"

namespace cortex {
namespace util {

class JointStatePublisher {
 public:
  JointStatePublisher(const std::vector<std::string>& joint_names,
                      const std::string& topic,
                      int queue_size)
      : joint_names_(joint_names), seq_(0) {
    ros::NodeHandle node_handle;
    pub_ = node_handle.advertise<sensor_msgs::JointState>(topic, queue_size);
  }

  void Publish(const math::State& state) {
    sensor_msgs::JointState msg;

    msg.header.seq = seq_++;
    msg.header.stamp = ros::Time::now();
    msg.name = joint_names_;
    msg.position = std::vector<double>(state.pos().data(), state.pos().data() + state.pos().size());
    msg.velocity = std::vector<double>(state.vel().data(), state.vel().data() + state.vel().size());

    pub_.publish(msg);
  }

 protected:
  ros::Publisher pub_;
  std::vector<std::string> joint_names_;
  int32_t seq_;
};

}  // namespace util
}  // namespace cortex
