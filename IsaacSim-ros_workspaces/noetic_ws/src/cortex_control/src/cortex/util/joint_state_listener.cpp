/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/util/joint_state_listener.h"

#include <atomic>
#include <mutex>

#include <ros/assert.h>

namespace cortex {
namespace util {

//------------------------------------------------------------------------------
// JointStateListener implementation
//------------------------------------------------------------------------------

void JointStateListener::Init(const std::string &topic,
                              const std::vector<std::string> &required_joints,
                              int poll_rate) {
  Init(topic, required_joints, true, poll_rate);
}

void JointStateListener::Init(const std::string &topic) { Init(topic, std::vector<std::string>()); }

void JointStateListener::Init(const std::string &topic, int poll_rate) {
  Init(topic, std::vector<std::string>(), poll_rate);
}

void JointStateListener::Init(const std::string &topic,
                              const std::vector<std::string> &required_joints) {
  Init(topic, required_joints, false, 0);
}

// This version is protected (internal).
void JointStateListener::Init(const std::string &topic,
                              const std::vector<std::string> &required_joints,
                              bool wait_until_available,
                              int poll_rate) {
  is_available_ = false;
  required_joints_ = required_joints;
  subscriber_ = node_handle_.subscribe(topic,
                                       10,  // Queue size.
                                       &JointStateListener::Callback,
                                       this);

  if (wait_until_available) {
    WaitUntilAvailable(poll_rate);
  }
}

void JointStateListener::InitWithZero(const std::string &topic,
                                      const std::vector<std::string> &required_joints) {
  required_joints_ = required_joints;
  subscriber_ = node_handle_.subscribe(topic,
                                       10,  // Queue size.
                                       &JointStateListener::Callback,
                                       this);
  for (uint32_t i = 0; i < required_joints_.size(); ++i) {
    current_state_map_[required_joints_[i]] = SingleJointState(0.0, 0.0, 0., ros::Time::now());
  }
  is_available_ = true;
}

void JointStateListener::SetRequiredJoints(const std::vector<std::string> &required_joints) {
  required_joints_ = required_joints;
}

void JointStateListener::WaitUntilAvailable(int poll_rate) const {
  ros::Rate rate(poll_rate);

  while (ros::ok() && !is_available()) {
    rate.sleep();
  }
}

void JointStateListener::Callback(const sensor_msgs::JointState &joint_states) {
  std::lock_guard<std::mutex> guard(msg_mutex_);
  auto n = joint_states.name.size();
  ROS_ASSERT(joint_states.position.size() == n);
  ROS_ASSERT(joint_states.velocity.size() == n);
  ROS_ASSERT(joint_states.effort.size() == 0 || joint_states.effort.size() == n);

  bool has_efforts = (joint_states.effort.size() > 0);

  for (uint32_t i = 0; i < n; ++i) {
    current_state_map_[joint_states.name[i]] =
        SingleJointState(joint_states.position[i],
                         joint_states.velocity[i],
                         has_efforts ? joint_states.effort[i] : 0.,
                         joint_states.header.stamp);
  }
  if (!is_available_) {
    // The method HasRequiredJoints(), which requires looping through the
    // required joints to see if they're ready, is only called during the
    // period of time when we're waiting for the first full set of
    // information to be available.
    is_available_ = HasRequiredJoints();
  }
}

const std::unordered_map<std::string, SingleJointState> &JointStateListener::current_state_map()
    const {
  return current_state_map_;
}

std::unordered_map<std::string, SingleJointState> JointStateListener::current_state_map_atomic()
    const {
  std::lock_guard<std::mutex> guard(msg_mutex_);
  return current_state_map_;
}

std::vector<double> JointStateListener::CurrentPositions(
    const std::vector<std::string> &names) const {
  std::lock_guard<std::mutex> guard(msg_mutex_);
  return ExtractNamedPositions(current_state_map_, names);
}

StampedState JointStateListener::CurrentState() const {
  std::lock_guard<std::mutex> guard(msg_mutex_);
  StampedState state(required_joints_.size());

  double min_time = 0.;
  for (uint32_t i = 0; i < required_joints_.size(); ++i) {
    const auto &name = required_joints_[i];
    auto access_iter = current_state_map_.find(name);
    ROS_ASSERT_MSG(access_iter != current_state_map_.end(),
                   "Required joint not found: %s", name.c_str());
    const auto &single_joint_state = access_iter->second;
    state.q(i) = single_joint_state.position;
    state.qd(i) = single_joint_state.velocity;
    state.u(i) = single_joint_state.effort;

    double time = single_joint_state.stamp.toSec();
    if (i == 0 || time < min_time) min_time = time;
  }

  state.time = min_time;

  return state;
}

bool JointStateListener::IsReady() const { return is_available(); }

//------------------------------------------------------------------------------
// Helper methods implementation
//------------------------------------------------------------------------------

bool JointStateListener::HasRequiredJoints() const {
  bool has_required_joints = true;
  std::cout << "Checking required joints: ";
  for (const auto &entry_name : required_joints_) {
    std::cout << "[" << entry_name << "(";
    if (current_state_map_.find(entry_name) == current_state_map_.end()) {
      std::cout << "-";
      has_required_joints = false;
    } else {
      std::cout << "+";
    }
    std::cout << ")]";
  }
  std::cout << "|" << std::endl;
  return has_required_joints;
}

std::unordered_map<std::string, SingleJointState> ToMap(
    const sensor_msgs::JointState &joint_states) {
  auto n = joint_states.name.size();
  ROS_ASSERT(joint_states.position.size() == n);
  ROS_ASSERT(joint_states.velocity.size() == n);
  ROS_ASSERT(joint_states.effort.size() == n);

  std::unordered_map<std::string, SingleJointState> js_map;
  for (uint32_t i = 0; i < n; ++i) {
    js_map[joint_states.name[i]] = SingleJointState(joint_states.position[i],
                                                    joint_states.velocity[i],
                                                    joint_states.effort[i],
                                                    joint_states.header.stamp);
  }
  return js_map;
}

std::vector<double> ExtractNamedPositions(
    const std::unordered_map<std::string, SingleJointState> &jstates,
    const std::vector<std::string> &names) {
  std::vector<double> positions;
  for (const auto &name : names) {
    auto access_iter = jstates.find(name);
    ROS_ASSERT(access_iter != jstates.end());
    positions.push_back(access_iter->second.position);
  }
  return positions;
}

std::vector<double> ExtractNamedPositions(const sensor_msgs::JointState &joint_states,
                                          const std::vector<std::string> &names) {
  return ExtractNamedPositions(ToMap(joint_states), names);
}

}  // namespace util
}  // namespace cortex
