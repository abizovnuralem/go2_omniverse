// Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <cortex/control/command_stream_interpolator.h>
#include <cortex_control/JointPosVelAccCommand.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace cortex {
namespace control {
namespace franka {

/**
 * \brief Joint position controller using cortex rmp control commands.
 *
 * This controller forwards with interpolation the received cortex
 * control commands to the robot's joint interface.
 */
class InterpolatedCommandStreamController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::PositionJointInterface> {
 public:
  /**
   * \brief Initializes the controller.
   *
   * \param robot_hardware handle to the robot's hardware abstraction
   * \param node_handle node handle instance
   */
  bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

  /**
   * \brief Initialization of the controller upon activation.
   *
   * \param time time at which the controller was activated
   */
  void starting(ros::Time const &time) override;

  /**
   * \brief Control update loop execution.
   *
   * \param time current time
   * \param period time elapsed since last call
   */
  void update(ros::Time const &time, ros::Duration const &period) override;

 private:
  /**
   * \brief Retrieves the current position from the joint handles.
   */
  Eigen::VectorXd current_position() const;

  /**
   * \brief Sends the robot's current pose to the robot.
   */
  void send_current_position();

  /**
   * \brief Sends the defined position to the robot's joints.
   *
   * \param q joint position to be sent to the robot
   */
  void send_position_command(const Eigen::VectorXd &q);

 private:
  std::shared_ptr<cortex::control::CommandStreamInterpolator> command_stream_interpolator_;

  bool initialize_blending_;
  ros::Time controller_time_;
  ros::Time start_time_;
  hardware_interface::PositionJointInterface *joint_interface_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Duration print_period_;
  ros::Time next_print_time_;
};

}  // namespace franka
}  // namespace control
}  // namespace cortex
