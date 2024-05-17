/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/control/joint_pos_vel_acc_command_publisher.h"

#include <vector>

#include <cortex_control/JointPosVelAccCommand.h>

namespace cortex {
namespace control {

JointPosVelAccCommandPublisher::JointPosVelAccCommandPublisher(
    const std::string& topic, bool stamp_header_with_controller_time)
    : stamp_header_with_controller_time_(stamp_header_with_controller_time),
      is_first_(true),
      next_id_(0) {
  topic_ = topic;
  ros::NodeHandle nh;
  joint_command_publisher_ = nh.advertise<cortex_control::JointPosVelAccCommand>(topic_, 10);
}

JointPosVelAccCommandPublisher::~JointPosVelAccCommandPublisher() {}

void JointPosVelAccCommandPublisher::Publish(uint64_t id,
                                             const ros::Time& t,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& qd,
                                             const Eigen::VectorXd& qdd) {
  cortex_control::JointPosVelAccCommand joint_command;
  if (stamp_header_with_controller_time_) {
    joint_command.header.stamp = t;
  } else {
    if (is_first_) {
      controller_time_offset_ = ros::Time::now() - t;
    }
    // We want to report the current time, but with the steadiness of the
    // controller time.
    joint_command.header.stamp = t + controller_time_offset_;
  }
  joint_command.id = id;

  if (is_first_) {
    // Usually this first message is missed by the interpolator (or it's
    // dropped because of syncing protocols), but even if it's used, the
    // interpolator won't use the period field because that's only used for
    // knowing the period between the previous point (there isn't one) and this
    // one.
    joint_command.period = ros::Duration(0.);
    is_first_ = false;
  } else {
    joint_command.period = (t - prev_t_);
  }

  joint_command.t = t;
  joint_command.names = joint_names;

  joint_command.q = std::vector<double>(q.data(), q.data() + q.size());
  joint_command.qd = std::vector<double>(qd.data(), qd.data() + qd.size());
  joint_command.qdd = std::vector<double>(qdd.data(), qdd.data() + qdd.size());

  joint_command_publisher_.publish(joint_command);

  // Updating the next_id_ member here means we can always set an ID once with
  // a call explicitly to this Publish(...) method and then use the ID-less
  // Publish(...) method to continue publishing sequential IDs from there.
  next_id_ = id + 1;
  prev_t_ = t;
}

void JointPosVelAccCommandPublisher::Publish(const ros::Time& t,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& qd,
                                             const Eigen::VectorXd& qdd) {
  // Note that this call automatically increments next_id.
  Publish(next_id_, t, joint_names, q, qd, qdd);
}

}  // namespace control
}  // namespace cortex
