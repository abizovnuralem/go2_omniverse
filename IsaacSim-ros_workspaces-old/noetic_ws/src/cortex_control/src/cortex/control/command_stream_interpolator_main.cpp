/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

// Runs a generic CommandStreamInterpolator without sending the commands to
// a physical robot. This enables visualizing the underlying interpolated
// commands to analyze interpolation techniques for specific problems. Note it
// doesn't use the NextCommand() interface, but directly jumps to the
// interpolations, so blending doesn't pollute early signals.

#include <iostream>

#include "cortex/control/builders.h"
#include "cortex/control/command_stream_interpolator.h"
#include "cortex/control/rmpflow_commanded_joints_listener.h"
#include "cortex/util/joint_state_listener.h"
#include "cortex/util/ros_util.h"
#include "cortex/util/yaml.h"

#include <gflags/gflags.h>
#include <ros/ros.h>

DEFINE_string(command_stream_interpolator_config,
              "package://cortex_control/config/command_stream_interpolator.yaml",
              "");
DEFINE_double(interpolated_control_rate_hz,
              500.,
              "Rate in Hz at which the low-level control will be sending "
              "commands. In this program, those commands are published on a "
              "new topic <command_topic>/interpolated.");
DEFINE_bool(use_rectified_cycles,
            false,
            "If true, rectifies the time stamp so they're always exactly a period "
            "apart. Otherwise (default), sets the time stamp to the current wall-clock "
            "time.");
DEFINE_bool(analysis_mode,
            false,
            "If true, runs in analysis mode. Doesn't use NextCommand() for interpolation "
            "between interpolated and desired when starting up. In general, you'll want to "
            "use NextCommand() in real controllers.");
DEFINE_bool(verbose, false, "Print extra messages.");

class MockControllerInterface {
 public:
  bool is_interpolator_active;

  MockControllerInterface(
      const std::shared_ptr<cortex::util::JointStateListener>& joint_state_listener)
      : is_interpolator_active(false), joint_state_listener_(joint_state_listener) {}

  Eigen::VectorXd GetMeasuredPositions() {
    if (is_interpolator_active) {
      // The interpolator is active, so as part of the protocol the joint state listener has
      // been set to listen to the same joints as found in the commands and the interpolator has
      // made sure those are available in the joint state listener. Therefore, we can return the
      // measure states from the listener.
      return joint_state_listener_->CurrentState().q;
    } else {
      // Otherwise, return a zero length vector. That will get the NextCommand() calls to return a
      // zero length vector as well.
      return Eigen::VectorXd(0);
    }
  }

 protected:
  std::shared_ptr<cortex::util::JointStateListener> joint_state_listener_;
};

int main(int argc, char** argv) {
  try {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "cortex_command_stream_interpolator");

    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    auto command_stream_interpolator_config = YAML::LoadFile(
        cortex::util::ExpandRosPkgRelPath(FLAGS_command_stream_interpolator_config));
    auto command_stream_interpolator = cortex::control::LoadCommandStreamInterpolatorFromYaml(
        command_stream_interpolator_config);

    command_stream_interpolator->Start();

    auto ros_topics = cortex::util::GetFieldOrDie(command_stream_interpolator_config, "ros_topics");
    auto joint_state_topic = cortex::util::GetOrDie<std::string>(ros_topics, "joint_state");
    auto command_topics = cortex::util::GetFieldOrDie(ros_topics, "rmpflow_commands");
    auto rmpflow_command_topic = cortex::util::GetOrDie<std::string>(command_topics, "command");
    cortex::control::RmpflowCommandedJointsListener rmpflow_commanded_joints_listener(
        rmpflow_command_topic, joint_state_topic);
    std::cout << "Waiting until joint states are available..." << std::endl;
    rmpflow_commanded_joints_listener.WaitUntilAvailable(30.);
    std::cout << "<done>" << std::endl;

    auto controller_interface =
        MockControllerInterface(rmpflow_commanded_joints_listener.joint_state_listener());

    auto rate_hz = FLAGS_interpolated_control_rate_hz;
    auto period = ros::Duration(1. / rate_hz);
    auto time = ros::Time::now();
    auto time_at_next_print = time;

    Eigen::VectorXd q_des;

    ros::Rate rate(rate_hz);
    bool is_interpolator_active = false;
    while (ros::ok()) {
      if (FLAGS_use_rectified_cycles) {
        time += period;
      } else {
        time = ros::Time::now();
      }

      if (FLAGS_analysis_mode) {
        // Analysis mode. Allows us to see the interpolated commands without the blending introduced
        // by NextCommand(). Controllers will typically want to use NextCommand().
        auto command = command_stream_interpolator->EvalAndStep(time);
        if (command) {
          q_des = command.commanded_position;
        }
      } else {
        // Standard mode. Usually you would send this next_command to the controller. Here, we just
        // use the internal functionality of the command stream interpolator to publish the command
        // on the specified interpolated commands topic.
        auto q_measured = controller_interface.GetMeasuredPositions();
        q_des = command_stream_interpolator->NextCommand(
            time, q_measured, &controller_interface.is_interpolator_active);
      }

      if (FLAGS_verbose && time >= time_at_next_print) {
        std::cout << "time = " << time << ", q_des = " << q_des.transpose() << std::endl;
        time_at_next_print += ros::Duration(.2);
      }

      rate.sleep();
    }

    std::cout << "<done>" << std::endl;
  } catch (const std::exception& ex) {
      std::cout << "Exception caught: " << ex.what() << std::endl;
  }

  return 0;
}
