// Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.

#include <iomanip>
#include <iostream>
#include <sstream>

#include <controller_interface/controller_base.h>
#include <cortex/math/interpolation/pos_vel_acc.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "cortex/control/builders.h"
#include "cortex/util/ros_util.h"  // TODO: verify has ExpandRosPkgRelPath()
#include "cortex/control/franka/interpolated_command_stream_controller.h"

namespace cortex {
namespace control {
namespace franka {

bool InterpolatedCommandStreamController::init(hardware_interface::RobotHW *robot_hardware,
                                               ros::NodeHandle &node_handle) {
  // Initialize connection to the robot and obtain joint handles
  joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();

  if (joint_interface_ == nullptr) {
    ROS_ERROR("InterpolatedCommandStreamController: Error getting position joint "
              "interface from hardware!");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("InterpolatedCommandStreamController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("InterpolatedCommandStreamController: Wrong number of joint names, got"
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  joint_handles_.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    try {
      joint_handles_[i] = joint_interface_->getHandle(joint_names[i]);
    } catch (hardware_interface::HardwareInterfaceException const &e) {
      ROS_ERROR_STREAM(
          "InterpolatedCommandStreamController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  auto command_stream_interpolator_config = YAML::LoadFile(
      cortex::util::ExpandRosPkgRelPath("package://cortex_control_franka/config/command_stream_interpolator.yaml"));
  command_stream_interpolator_ = cortex::control::LoadCommandStreamInterpolatorFromYaml(
      command_stream_interpolator_config);
  return true;
}

void InterpolatedCommandStreamController::starting(ros::Time const &time) {
  initialize_blending_ = true;
  print_period_ = ros::Duration(1.);
  start_time_ = time;
  controller_time_ = time;
  next_print_time_ = time;
  command_stream_interpolator_->Start();
}

Eigen::VectorXd InterpolatedCommandStreamController::current_position() const {
  Eigen::VectorXd q(joint_handles_.size());
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    q[i] = joint_handles_[i].getPosition();
  }
  return q;
}

void InterpolatedCommandStreamController::send_current_position() {
  send_position_command(current_position());
}

void InterpolatedCommandStreamController::send_position_command(Eigen::VectorXd const &q) {
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    joint_handles_[i].setCommand(q[i]);
  }
}

void InterpolatedCommandStreamController::update(ros::Time const &time,
                                                     ros::Duration const &period) {
  // Update time information.
  //
  // WARNING: This method of accumulation into a duration using the period
  //          provided to the method is the only way of handling time
  //          that works, all other options will result in the robot
  //          producing motor noises during motion.
  controller_time_ += period;
  bool is_interpolator_active;
  send_position_command(command_stream_interpolator_->NextCommand(
      controller_time_, current_position(), &is_interpolator_active));

  if (time >= next_print_time_) {
    std::cout << std::setprecision(10) << "[franka] time: " << (time - start_time_).toSec()
              << ", control_time: " << (controller_time_ - start_time_).toSec()
              << ", now: " << (ros::Time::now() - start_time_).toSec()
              << ", period: " << period.toSec() << std::endl;

    next_print_time_ += print_period_;
  }
}

}  // namespace franka
}  // namespace control
}  // namespace cortex

PLUGINLIB_EXPORT_CLASS(cortex::control::franka::InterpolatedCommandStreamController,
                       controller_interface::ControllerBase)
