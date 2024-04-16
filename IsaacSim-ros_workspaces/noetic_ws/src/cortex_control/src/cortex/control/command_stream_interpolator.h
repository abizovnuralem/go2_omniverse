/**
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in
 * and to this software, related documentation and any modifications thereto.  Any use,
 * reproduction, disclosure or distribution of this software and related documentation without an
 * express license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <cortex_control/JointPosVelAccCommand.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "cortex/math/interpolation/interpolator.h"

namespace cortex {
namespace control {

enum class ControllerState {

  // The controller hasn't started yet so we should ignore any incoming commands so we don't start
  // processing state changes prematurely.
  StartingController = 0,

  // The controller's eval calls have started. We need to wait on the backend to start. (The backend
  // may already be started, in which case it'll immediately transition once the next incoming
  // command is received).
  WaitingOnBackend,

  // We need to sync the backend with the current state of the robot by suppressing it briefly.
  // Suppression automatically sets the backend to latest measured state from the robot. We remain
  // in this state until we've detected we're no longer receiving messages from the backend.
  SyncingBackend,

  // After we've detected the backend suppression has been successful, we stop suppressing and
  // transition to initializing the interpolator.  The next incoming command will be used to
  // initialize the interpolator. The NextCommand() interface can be used to blend between the
  // measured state of the robot and the interpolated command for a blending duration specified on
  // initialization.
  InitializingInterpolator,

  // Once the interpolator is initialized, we're operation and running as expected.
  Operational
};

}  // namespace control
}  // namespace cortex

namespace cortex {
namespace control {

// Enables usage of the following form:
//
//   auto suppressor = std::make_shared<cortex::control::CommandSuppressor>(topic, rate_hz);
//   suppressor->StartSuppression();
//   ros::Duration(2.).sleep();
//   suppressor->StopSuppression();
//
// Internally, it constantly sends suppression messages at the specified rate and switches from
// sending std_msgs::String("1") for suppression to st_msgs::String("0") when not suppressing.

class CommandSuppressor {
 public:
  static std::string default_topic;
  static double default_rate_hz;

  // Defaults topic to default_topic and the publication rate to default_rate_hz.
  CommandSuppressor() : CommandSuppressor(default_topic, default_rate_hz) {}

  // Initialize to publish on the specified topic at the specified rate.  Constantly publishes in a
  // separate thread.
  CommandSuppressor(const std::string& topic, double rate_hz);

  ~CommandSuppressor();

  void StartSuppressing() { is_suppressing_ = true; }
  void StopSuppressing() { is_suppressing_ = false; }

 protected:
  void Run();

  std::atomic_bool is_suppressing_;  // True when suppressing.
  std::atomic_bool is_running_;      // Set to false to stop the thread.

  std::string topic_;               // Topic it'll publish on.
  double rate_hz_;                  // Rate at which it'll publish.
  ros::Publisher suppression_pub_;  // The publisher itself.

  std::thread run_thread_;  // Thread running the constant publication stream.
};

// Interpolator receiving a stream of cortex commands and reconstructing the integral curve they
// describe using a quintic interpolator. It's assumed that Eval() is called at a regular control
// rate; the eval times are used as a clock for the system.
class CommandStreamInterpolator {
 public:
  static const double default_blending_duration;
  static const double default_backend_timeout;
  static const double default_time_between_interp_pubs;

  // A command is a commanded position plus return information on the availability from the Eval()
  // method. This enables the following syntax
  //
  //   auto command = stream_interpolator->Eval(...);
  //   if (command) {
  //      Send(command);
  //   }
  //
  // There's a Command::Unavailable() static convenince method for retrieving a generic unavailable
  // command.
  struct Command {
    bool is_available;
    Eigen::VectorXd commanded_position;

    Command(const Eigen::VectorXd& commanded_position)
        : is_available(true), commanded_position(commanded_position) {}
    Command() : is_available(false) {}

    // Enables checking boolean truth value of the command to see whether
    // or not it's available.
    operator bool() { return is_available; }

    static Command Unavailable() { return Command(); }
  };

  // By default doesn't use the smoothing interpolator.
  bool Init(const ros::Duration& interpolator_lookup_delay_buffer,
            const std::string& cortex_command_topic,
            ros::Duration blending_duration = ros::Duration(default_blending_duration)) {
    return Init(interpolator_lookup_delay_buffer, false, cortex_command_topic);
  }

  // interpolator_lookup_delay_buffer is how far in the past to look up interpolated values to
  // accommodate possible jitter.
  //
  // use_smoothing_interpolator: if true, uses a smoothing interpolator.  Otherwise, uses a basic
  // quintic interpolator.
  //
  // cortex_command_topic: topic on which cortex_control::JointPosVelAccCommand messages are broadcast.
  //
  // blending_duration: how long to blend for during start up when using NextCommand().
  bool Init(const ros::Duration& interpolator_lookup_delay_buffer,
            bool use_smoothing_interpolator,
            const std::string& cortex_command_topic,
            ros::Duration blending_duration = ros::Duration(default_blending_duration),
            double backend_timeout = default_backend_timeout);
  bool Init(const ros::Duration& interpolator_lookup_delay_buffer,
            bool use_smoothing_interpolator,
            const std::string& cortex_command_topic,
            const std::string& cortex_command_ack_topic,
            const std::string& cortex_command_suppress_topic,
            const std::string& cortex_command_interpolated_topic,
            ros::Duration blending_duration = ros::Duration(default_blending_duration),
            double backend_timeout = default_backend_timeout);

  void Start();

  // Returns true if enough time has passed since the last cortex command callback to designate the
  // backend as having been stopped or successfully suppressed.
  bool IsBackendTimedOut(const ros::Time& time) const;

  // Evaluate the interpolator at the specified time index. Time indices should be monotonically
  // increasing, and calling this method steps the protocol. The Command is flagged as not available
  // until the protocol is in the Operational state.
  Command EvalAndStep(const ros::Time& time);

  // Internally calls EvalAndStep(time), but handles unavailable commands cleanly and smoothly
  // interpolates as needed to create a smooth transition to interpolation on startup.
  //
  // Automatically switches between returning q_measured when the interpolator isn't ready, blending
  // between q_measured and the interpolated values for a predefined duration (blend_duration, set
  // at initialization), and fully returning the interpolated values once blending is complete.  It
  // is recommended that this method be used for smooth transitioning to interpolated command stream
  // control.
  //
  // q_measured can be smaller in length than the internal interpolated commands.  In that case,
  // just the first q_measured.size() joint commands are used, and the returned command vector is of
  // length q_measured.size().
  Eigen::VectorXd NextCommand(const ros::Time& time,
                              const Eigen::VectorXd& q_measured,
                              bool* is_interpolator_active = nullptr);

 private:
  void CommandCallback(const cortex_control::JointPosVelAccCommand& msg);

  // Add the command in the given command_msg to the interpolator. The command messages were meant
  // to describe waypoints along an integral curve, so their command_msg.t time stamp is a rectified
  // (jitter free) time stamp that can be used for interpolation.
  void AddPointToInterpolator(const cortex_control::JointPosVelAccCommand& command_msg);

  // Add the given  interpolation point to the interolator at the given time.
  void AddPointToInterpolator(const ros::Time& time, const cortex::math::PosVelAccXd& point);

  // This method error checks on the state of the controller and shifts the time point to index the
  // interpolator correctly. The time input should be controller time.
  //
  // Returns the result in eval_point.
  //
  // If there's an error, returns false (and if the optional error_str is available, sets the error
  // string). Otherwise, return true on success.
  bool EvalInterpolator(const ros::Time& time,
                        cortex::math::PosVelAccXd& eval_point,
                        std::string* error_str = nullptr) const;

  // Publish the given interpolated point as a
  //
  //   cortex_control::JointPosVelAccCommand
  //
  // on <joint_command_topic>/interpolated.
  void PublishInterpolatedPoint(const ros::Time& time, const cortex::math::PosVelAccXd& point) const;

  // Resets the interpolator to the initial state. One should always call this method for any event
  // that transitions the system back to the WaitingOnBackend state.
  void ResetInterpolator();

  // Protects all members between calls to Eval() and CommandCallback().
  std::mutex mutex_;

  // Time at the most recent eval. This enables syncing the clocks between Eval() and Callback().
  ros::Time last_eval_time_;

  // Number of seconds in the past to evaluate the interpolator. The interpolator is effectively
  // evaluated as interpolator->Eval(<now> - <delay_buffer>), ...); There are some details about
  // syncing the clocks between incoming commands and controllers Eval() time, but the gist of it is
  // that new incoming points are added at time <now> and we evaluate at <now> - <delay_buffer>.
  ros::Duration interpolator_lookup_delay_buffer_;

  // Time of the incoming command when the interpolator was initialized (this is actually the second
  // point in the interpolator -- we actually step that back by buffer delay and interpolate from
  // the current position to this initial incoming command).
  ros::Time eval_time_at_interpolator_start_;
  ros::Time command_time_at_interpolator_start_;
  ros::Duration control_time_offset_from_now_;

  // The underlying quintic interpolator.
  std::shared_ptr<cortex::math::Interpolator<Eigen::VectorXd>> interp_;

  // Current state of the stream interpolator. This orchestrates the sync protocol with the cortex
  // commander.
  ControllerState state_;

  // The time stamp of the Eval() call when the latest incoming command was received.
  ros::Time eval_time_at_last_callback_;

  // ROS publishers and subscribers.
  ros::Subscriber cortex_command_sub_;
  ros::Publisher interpolated_command_pub_;
  ros::Publisher cortex_command_time_pub_;
  cortex_control::JointPosVelAccCommand latest_command_msg_;

  // A command suppressor used during the backend sync to sync the backend with the measured
  // joint states.
  std::shared_ptr<CommandSuppressor> command_suppressor_;

  // If true, uses an auto smoothing interpolator when ResetInterpolator() is called.
  bool use_smoothing_interpolator_;

  // These three members are used to coordinate blending during
  ros::Time blending_start_time_;
  bool start_blending_;
  ros::Duration blending_duration_;

  ros::Time next_print_time_;
  ros::Duration print_period_;

  double time_offset_;
  double momentum_;
  double time_between_interp_pubs_;
  mutable ros::Time time_at_last_pub_;

  double backend_timeout_;
};

}  // namespace control
}  // namespace cortex
