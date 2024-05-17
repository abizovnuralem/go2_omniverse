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

#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/ros.h>

namespace cortex {
namespace control {

/*!\brief Abstract class representing the base of a oint position, velocity,
 * and acceleration command publisher. The topic name of the publisher is
 * defined as  ns + "joint_command".
 */
class JointPosVelAccCommandPublisher {
 public:
  /*!\brief Creates a JointPosVelAccCommandPublisher under the given name
   * space *ns*. The topic name of the publisher is defined as  ns +
   * "joint_command".
   *
   * There are two time stamps in each JointPosVelAccCommand message, one in
   * the header and another as an explicit field t. The explicit field is
   * always set to be the controller time (with each message exactly a period
   * duration between), but by default (if stamp_header_with_controller_time is
   * false) the header contains the wall clock time so we can see the jitter in
   * the calculation using tools like rqt_plot. If
   * stamp_header_with_controller_time is true, that header stamp is also set
   * to the controller time so that becomes observable in plotters.
   */
  JointPosVelAccCommandPublisher(const std::string& ns,
                                 bool stamp_header_with_controller_time = false);

  /*!\brief Default virtual destructor
   */
  ~JointPosVelAccCommandPublisher();

  /*!\brief Publishes the position, velocity, and acceleration command. Each
   * call to this method sets the id counter to the provided value, so
   * subsequent calls to the id-less API will increment from this id.
   *
   * \param time The time stamp of this command.
   * \param id the sequence id of this command.
   * \param joint_names Joint names vector. This vector must have the same
   * order as q qd, and qdd, i.e. the i-th name must correspond to the i-th q,
   * qd, qdd values.
   * \param q Joint position values
   * \param qd Joint velocity values
   * \param qdd Joint acceleration values
   */
  virtual void Publish(uint64_t id,
                       const ros::Time& t,
                       const std::vector<std::string>& joint_names,
                       const Eigen::VectorXd& q,
                       const Eigen::VectorXd& qd,
                       const Eigen::VectorXd& qdd);

  /*!\brief This version automatically creates the sequence id, starting from
   * zero and incrementing once for each call.
   */
  void Publish(const ros::Time& t,
               const std::vector<std::string>& joint_names,
               const Eigen::VectorXd& q,
               const Eigen::VectorXd& qd,
               const Eigen::VectorXd& qdd);

  const std::string& topic() const { return topic_; }

 protected:
  bool stamp_header_with_controller_time_;
  ros::Publisher joint_command_publisher_;
  ros::Duration controller_time_offset_;

  bool is_first_;
  ros::Time prev_t_;
  uint64_t next_id_;

  std::string topic_;
};

}  // namespace control
}  // namespace cortex
