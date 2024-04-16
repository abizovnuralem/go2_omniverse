/* Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in
 * and to this software, related documentation and any modifications thereto.  Any use,
 * reproduction, disclosure or distribution of this software and related documentation without an
 * express license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/control/rmpflow_commanded_joints_listener.h"

namespace cortex {
namespace control {

RmpflowCommandedJointsListener::RmpflowCommandedJointsListener(
    const std::string& rmpflow_commands_topic, const std::string& joint_state_topic)
    : rmpflow_commands_listener_(rmpflow_commands_topic, 1),
      joint_state_listener_(std::make_shared<util::JointStateListener>()) {
  joint_state_listener_->Init(joint_state_topic);

  rmpflow_commands_listener_.RegisterCallback([&](const auto& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    joint_state_listener_->SetRequiredJoints(msg.names);
  });
}

bool RmpflowCommandedJointsListener::IsAvailable() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return is_set_ && joint_state_listener_->is_available();
}

void RmpflowCommandedJointsListener::WaitUntilAvailable(double poll_rate) const {
  ros::Rate rate(poll_rate);

  while (ros::ok() && !IsAvailable()) {
    rate.sleep();
  }
}

const std::shared_ptr<util::JointStateListener>
RmpflowCommandedJointsListener::joint_state_listener() const {
  return joint_state_listener_;
}
}  // namespace control
}  // namespace cortex
