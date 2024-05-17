/* Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in
 * and to this software, related documentation and any modifications thereto.  Any use,
 * reproduction, disclosure or distribution of this software and related documentation without an
 * express license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <mutex>

#include <cortex_control/JointPosVelAccCommand.h>

#include "cortex/util/joint_state_listener.h"
#include "cortex/util/ros_message_listener.h"

namespace cortex {
namespace control {

// A wrapper around the joint state listener ensuring that we listen to the same joints we're
// controlling with the RMPflow commander's commands.
//
// Listens to the RMPflow commander's commands as well as the joint state topic. Once we receive
// the first command, we register the joint names with the joint state listener as required joints.
// The IsAvailable() method (or WaitUntilAvailable()) can then be used to check whether the joint
// state listener is ready and has measured values for each of those named joints.
class RmpflowCommandedJointsListener {
 public:
  RmpflowCommandedJointsListener(const std::string& rmpflow_commands_topic,
                                 const std::string& joint_state_topic);

  bool IsAvailable() const;
  void WaitUntilAvailable(double poll_rate) const;

  const std::shared_ptr<util::JointStateListener> joint_state_listener() const;

 protected:
  mutable std::mutex mutex_;

  util::RosMessageListener<cortex_control::JointPosVelAccCommand> rmpflow_commands_listener_;
  bool is_set_;
  std::shared_ptr<util::JointStateListener> joint_state_listener_;
};

}  // namespace control
}  // namespace cortex
