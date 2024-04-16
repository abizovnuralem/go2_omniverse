/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include "cortex/util/state_listener.h"

#include <atomic>
#include <mutex>
#include <unordered_map>

#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace cortex {
namespace util {

/*!\brief Contains information about the state of a single joint. Includes
 * the time stamp of the message that last updated the joint.
 */
struct SingleJointState {
  double position;
  double velocity;
  double effort;
  ros::Time stamp;

  SingleJointState() {}
  SingleJointState(double pos, double vel, double eff, const ros::Time &stamp)
      : position(pos), velocity(vel), effort(eff), stamp(stamp) {}
};

typedef std::unordered_map<std::string, SingleJointState> JointStateMap;

/*!\brief A very simple joint state listener that records the latest joint
 * state information in an unordered map mapping the joint name to the most
 * recent SingleJointState information.
 *
 * It's necessary to process the information this way rather than simply
 * recording the joint state messages because there's no guarantee that each
 * joint state message contains information about all of the joints. (This, for
 * instance, is an issue with Baxter.)
 *
 * This class is thread safe.
 */
class JointStateListener : public StateListener {
 public:
  JointStateListener() = default;

  /*!\brief Initialize to listen on the specified topic for the given required joints. Blocks
   * waiting for for the joints to be available before returning, polling at the given poll rate.
   */
  void Init(const std::string &topic,
            const std::vector<std::string> &required_joints,
            int poll_rate);

  /*!\brief Initialize to listen on the specified topic for the given required joints. This version
   * does not block. Users must check explicitly is_available() before accessing.
   */
  void Init(const std::string &topic, const std::vector<std::string> &required_joints);

  void Init(const std::string &topic, int poll_rate);
  void Init(const std::string &topic);

  /*!\brief Initializes the listener with 0.0 as the a default joint state
   * values. The listener becomes immediately available.
   */

  void InitWithZero(const std::string &topic, const std::vector<std::string> &required_joints);

  /*!\brief Set the required joints (often used in conjunction with
   * Init(topic, poll_rate)). Optionally set wait_until_available to true to
   * block until they're available.
   */
  void SetRequiredJoints(const std::vector<std::string> &required_joints);

  /*!\brief Wait until at information for at least the specified
   * required_joints is available.
   */
  void WaitUntilAvailable(int poll_rate) const;

  /*!\brief Returns true if the required joints are available.
   */
  bool is_available() const { return is_available_; }

  /*!\brief This variant of the accessor is not atomic. It performs no
   * locking.
   */
  const JointStateMap &current_state_map() const;

  /*!\brief This variant is atomic. The only way to ensure no race condition
   * is to fully copy the internal state out.
   */
  JointStateMap current_state_map_atomic() const;

  /*!\brief Returns a vector of position values for the given named joints
   * retaining the specified joint order.
   */
  std::vector<double> CurrentPositions(const std::vector<std::string> &names) const;

  /*!\brief Returns the state of the system stamped with the minimum time
   * stamp (oldest) of all the active joints. The state is the positions,
   * velocities, and accelerations of the active joints.
   */
  StampedState CurrentState() const;

  /*!\brief Accessors implementing the StateListener API.
   */
  StampedState State() const override { return CurrentState(); }
  bool IsReady() const override;

  /*!\brief Accessor for the vector of required joints.
   */
  const std::vector<std::string> &required_joints() const { return required_joints_; }

 protected:
  /*!\brief Initialize to listen on the specified topic for the given required joints. If
   * wait_until_available is true, blocks waiting for for the joints to be available before
   * returning, polling at the given poll rate.
   */
  void Init(const std::string &topic,
            const std::vector<std::string> &required_joints,
            bool wait_until_available,
            int poll_rate);

  // Calls to this method should be externally protected through a msg_mutex_
  // lock.
  bool HasRequiredJoints() const;

  /*!\brief Callback consuming sensor_msgs::JointState messages. Writes the
   * information into the internal current_state_map_.
   */
  void Callback(const sensor_msgs::JointState &joint_states);

  mutable std::mutex msg_mutex_;
  std::vector<std::string> required_joints_;
  std::unordered_map<std::string, SingleJointState> current_state_map_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;

  std::atomic_bool is_available_;
};

//------------------------------------------------------------------------------
// Helper methods
//------------------------------------------------------------------------------

std::unordered_map<std::string, SingleJointState> ToMap(
    const sensor_msgs::JointState &joint_states);

std::vector<double> ExtractNamedPositions(
    const std::unordered_map<std::string, SingleJointState> &jstates,
    const std::vector<std::string> &names);

std::vector<double> ExtractNamedPositions(const sensor_msgs::JointState &joint_states,
                                          const std::vector<std::string> &names);
}  // namespace util
}  // namespace cortex
