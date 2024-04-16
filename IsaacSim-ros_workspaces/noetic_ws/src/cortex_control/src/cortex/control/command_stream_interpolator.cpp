/**
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in
 * and to this software, related documentation and any modifications thereto.  Any use,
 * reproduction, disclosure or distribution of this software and related documentation without an
 * express license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#include "cortex/control/command_stream_interpolator.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>

#include "cortex/math/interpolation/cubic_position_interpolator.h"
#include "cortex/math/interpolation/incremental_interpolator.h"
#include "cortex/math/interpolation/pos_vel_acc.h"
#include "cortex/math/interpolation/quartic_interpolator.h"
#include "cortex/math/interpolation/smoothing_incremental_interpolator.h"
#include "cortex_control/CortexCommandAck.h"

namespace cortex {
namespace control {

inline std::ostream& operator<<(std::ostream& os, ControllerState state) {
  using namespace cortex::control;
  switch (state) {
    case ControllerState::StartingController:
      os << "ControllerState::StartingController";
      break;
    case ControllerState::WaitingOnBackend:
      os << "ControllerState::WaitingOnBackend";
      break;
    case ControllerState::SyncingBackend:
      os << "ControllerState::SyncingBackend";
      break;
    case ControllerState::InitializingInterpolator:
      os << "ControllerState::InitializingInterpolator";
      break;
    case ControllerState::Operational:
      os << "ControllerState::Operational";
      break;
    default:
      os << "ControllerState::<unknown>";
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, CommandStreamInterpolator::Command& command) {
  if (command) {
    os << "[" << command.commanded_position.transpose() << "]";
  } else {
    os << "<unavailable>";
  }
  return os;
}

std::string CommandSuppressor::default_topic = "/robot/command_suppression/right";
double CommandSuppressor::default_rate_hz = 30.;

CommandSuppressor::CommandSuppressor(const std::string& topic, double rate_hz)
    : topic_(topic), rate_hz_(rate_hz) {
  is_running_ = true;
  is_suppressing_ = false;

  ros::NodeHandle node_handle;
  suppression_pub_ = node_handle.advertise<std_msgs::Bool>(topic_, 10);

  run_thread_ = std::thread(&CommandSuppressor::Run, this);
}

CommandSuppressor::~CommandSuppressor() {
  is_running_ = false;
  run_thread_.join();
}

void CommandSuppressor::Run() {
  ros::Rate rate(rate_hz_);
  while (ros::ok() && is_running_) {
    std_msgs::Bool msg;
    if (is_suppressing_) {
      msg.data = true;
    } else {
      msg.data = false;
    }
    suppression_pub_.publish(msg);
    rate.sleep();
  }
}

const double CommandStreamInterpolator::default_blending_duration = 2.;
const double CommandStreamInterpolator::default_backend_timeout = .5;
const double CommandStreamInterpolator::default_time_between_interp_pubs = 1. / 60;  // 60 hz

bool CommandStreamInterpolator::Init(const ros::Duration& interpolator_lookup_delay_buffer,
                                     bool use_smoothing_interpolator,
                                     const std::string& cortex_command_topic,
                                     ros::Duration blending_duration,
                                     double backend_timeout) {
  return Init(interpolator_lookup_delay_buffer,
              use_smoothing_interpolator,
              cortex_command_topic,
              cortex_command_topic + "/ack",
              cortex_command_topic + "/suppress",
              cortex_command_topic + "/interpolated",
              blending_duration,
              backend_timeout);
}

bool CommandStreamInterpolator::Init(const ros::Duration& interpolator_lookup_delay_buffer,
                                     bool use_smoothing_interpolator,
                                     const std::string& cortex_command_topic,
                                     const std::string& cortex_command_ack_topic,
                                     const std::string& cortex_command_suppress_topic,
                                     const std::string& cortex_command_interpolated_topic,
                                     ros::Duration blending_duration,
                                     double backend_timeout) {
  interpolator_lookup_delay_buffer_ = interpolator_lookup_delay_buffer;
  use_smoothing_interpolator_ = use_smoothing_interpolator;
  blending_duration_ = blending_duration;
  backend_timeout_ = backend_timeout;
  time_between_interp_pubs_ = default_time_between_interp_pubs;

  ros::NodeHandle node_handle;

  // Create pub-subs.
  cortex_command_sub_ = node_handle.subscribe(
      cortex_command_topic, 1, &CommandStreamInterpolator::CommandCallback, this);
  interpolated_command_pub_ =
      node_handle.advertise<cortex_control::JointPosVelAccCommand>(cortex_command_interpolated_topic, 10);
  cortex_command_time_pub_ =
      node_handle.advertise<cortex_control::CortexCommandAck>(cortex_command_ack_topic, 10);

  // Create the suppressor with defaults.
  command_suppressor_ = std::make_shared<CommandSuppressor>(
      cortex_command_suppress_topic, CommandSuppressor::default_rate_hz);

  return true;
}

void CommandStreamInterpolator::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::cout << "<starting_controller>" << std::endl;
  state_ = ControllerState::StartingController;
}

bool CommandStreamInterpolator::IsBackendTimedOut(const ros::Time& time) const {
  auto delta = (time - eval_time_at_last_callback_).toSec();
  return delta >= backend_timeout_;
}

CommandStreamInterpolator::Command CommandStreamInterpolator::EvalAndStep(
    const ros::Time& time) {
  std::lock_guard<std::mutex> lock(mutex_);

  last_eval_time_ = time;

  // Check state transitions.
  if (state_ == ControllerState::StartingController) {
    control_time_offset_from_now_ = ros::Time::now() - time;

    ResetInterpolator();
    std::cout << "<starting> --> <waiting_on_backend>" << std::endl;
    state_ = ControllerState::WaitingOnBackend;
  } else if (state_ == ControllerState::WaitingOnBackend) {
    // The callback switches us out of this one.
  } else if (state_ == ControllerState::SyncingBackend) {
    // If we've stopped receiving messages from the backend, stop suppressing and transition to
    // initializing the interpolator.
    if (IsBackendTimedOut(time)) {
      std::cout << "<syncing_backend> --> <initializing_interpolator>" << std::endl;
      state_ = ControllerState::InitializingInterpolator;
      command_suppressor_->StopSuppressing();
    }

  } else if (state_ == ControllerState::InitializingInterpolator) {
    // The callback switches us out of this one.
  } else if (state_ == ControllerState::Operational) {
    // We're good to go. We'll just execute until it looks like we've lost communication with the
    // backend.
    if (IsBackendTimedOut(time)) {
      ResetInterpolator();
      std::cout << "<operational> --> <waiting_on_backend>" << std::endl;
      state_ = ControllerState::WaitingOnBackend;
    }
  }

  // Process states.
  if (state_ == ControllerState::StartingController) {
    // we should immediately transition to waiting on backend.
    std::cerr << "There's something wrong. We should never get here. Diagnose "
              << "immediately.";
    throw std::runtime_error("Bad state in CommandStreamInterpolator");
    return Command::Unavailable();

  } else if (state_ == ControllerState::WaitingOnBackend) {
    // Just wait until we start receiving messages.
    return Command::Unavailable();

  } else if (state_ == ControllerState::SyncingBackend) {
    // We're currently suppressing in a separate thread using the command_suppressor_.
    // Otherwise, do nothing.
    return Command::Unavailable();

  } else if (state_ == ControllerState::InitializingInterpolator) {
    time_at_last_pub_ = time;
    // This is handled by the callback.
    return Command::Unavailable();

  } else if (state_ == ControllerState::Operational) {
    auto lookup_time = time - interpolator_lookup_delay_buffer_;
    if (lookup_time < eval_time_at_interpolator_start_) {
      return Command::Unavailable();
    }

    // Get interpolated command.
    cortex::math::PosVelAccXd eval_point;
    std::string error_str;
    if (!EvalInterpolator(lookup_time, eval_point, &error_str)) {
      ROS_WARN_STREAM("[cortex] " << error_str);
      return Command::Unavailable();
    }
    PublishInterpolatedPoint(time, eval_point);
    return Command(eval_point.x);
  } else {
    std::cerr << "Unrecognized state: " << state_;
    throw std::runtime_error("Bad state in CommandStreamInterpolator");
  }
}

Eigen::VectorXd CommandStreamInterpolator::NextCommand(const ros::Time& time,
                                                       const Eigen::VectorXd& q_measured,
                                                       bool* is_interpolator_active) {
  auto command = EvalAndStep(time);
  if (is_interpolator_active) {
    *is_interpolator_active = static_cast<bool>(command);
  }
  if (command) {
    if (start_blending_) {
      blending_start_time_ = time;
      start_blending_ = false;
    }

    auto elapse = (time - blending_start_time_).toSec();
    auto blend_duration = blending_duration_.toSec();

    Eigen::VectorXd q_des = command.commanded_position.head(q_measured.size());
    if (elapse < blend_duration) {
      auto alpha = elapse / blend_duration;  // Goes linearly from zero to one.
      alpha *= alpha;                        // Quadratic increase.
      q_des = alpha * q_des + (1. - alpha) * q_measured;
    }
    return q_des;
  } else {
    start_blending_ = true;
    return q_measured;
  }
}

void CommandStreamInterpolator::AddPointToInterpolator(
    const cortex_control::JointPosVelAccCommand& command_msg) {
  cortex::math::PosVelAccXd point;
  point.x = Eigen::Map<const Eigen::VectorXd>(command_msg.q.data(), command_msg.q.size());
  point.xd = Eigen::Map<const Eigen::VectorXd>(command_msg.qd.data(), command_msg.qd.size());
  point.xdd = Eigen::VectorXd::Zero(point.x.size());  // Accelerations not used by interpolator.
  AddPointToInterpolator(command_msg.t, point);
}

void CommandStreamInterpolator::AddPointToInterpolator(const ros::Time& time,
                                                       const cortex::math::PosVelAccXd& point) {
  std::string error_str;
  if (!interp_->AddPt(  // We add the first point slightly in the past.
          (time - command_time_at_interpolator_start_).toSec(),
          point,
          &error_str)) {
    ROS_ERROR_STREAM("[monolithic]: " << error_str);
  }
}

bool CommandStreamInterpolator::EvalInterpolator(const ros::Time& time,
                                                 cortex::math::PosVelAccXd& eval_point,
                                                 std::string* error_str) const {
  if (state_ != ControllerState::Operational) {
    if (error_str) {
      std::stringstream ss;
      ss << "Attempting to evaluate interpolator before reaching "
            "ControllerState::Operational. Current state: "
         << cortex::control::ControllerState::Operational;

      *error_str = ss.str();
    }
    return false;
  }

  return interp_->Eval(
      (time - eval_time_at_interpolator_start_).toSec() + time_offset_, eval_point, error_str);
}

void CommandStreamInterpolator::PublishInterpolatedPoint(
    const ros::Time& time, const cortex::math::PosVelAccXd& point) const {
  if ((time - time_at_last_pub_).toSec() >= time_between_interp_pubs_) {
    cortex_control::JointPosVelAccCommand command_msg;
    command_msg.header.stamp = time + control_time_offset_from_now_;
    command_msg.names = latest_command_msg_.names;
    command_msg.q = std::vector<double>(point.x.data(), point.x.data() + point.x.size());
    command_msg.qd = std::vector<double>(point.xd.data(), point.xd.data() + point.xd.size());
    command_msg.qdd = std::vector<double>(point.xdd.data(), point.xdd.data() + point.xdd.size());
    interpolated_command_pub_.publish(command_msg);
    time_at_last_pub_ = time;
  }
}

void CommandStreamInterpolator::CommandCallback(
    const cortex_control::JointPosVelAccCommand& command_msg) {
  if (command_msg.period == ros::Duration(0.)) {
    std::cout << "<rejecting first message sent by backend>" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_command_msg_ = command_msg;

  // While syncing the backend (state ControllerState::SyncingBackend) we suppress commands so
  // callbacks stop. We need to check how much time's elapsed since the last callback (and it needs
  // to be comparable to eval times, hence we set it to last_eval_time_). Note it's important that
  // we check time since the last callback and not time since the state transition because
  // transitioning to that state causes suppression commands to be sent to the backend. We want to
  // measure how much time has elapsed since the commands actually start being suppressed, not since
  // we started *trying* to suppress commands.
  eval_time_at_last_callback_ = last_eval_time_;

  if (state_ == ControllerState::StartingController) {
    return;  // Don't do anything until Update has been called once.
  } else if (state_ == ControllerState::WaitingOnBackend) {
    // The fact we're in the callback means we're up and running. Transition to syncing the backend.
    std::cout << "<waiting_on_backend> --> <syncing_backend>" << std::endl;
    state_ = ControllerState::SyncingBackend;
    command_suppressor_->StartSuppressing();
    // Until the backend's synced, we don't want to be interpolating points.
    return;
  } else if (state_ == ControllerState::SyncingBackend) {
    return;  // Still syncing.
  } else if (state_ == ControllerState::InitializingInterpolator) {
    // This aligns the interpolator's start time (command_msg.t) with the last controller time at
    // the last Eval.
    eval_time_at_interpolator_start_ = last_eval_time_;
    command_time_at_interpolator_start_ = command_msg.t;

    time_offset_ = 0.;
    momentum_ = 0.;

    // Now add the current commanded target at the current time and we're ready to start
    // interpolating.
    AddPointToInterpolator(command_msg);

    std::cout << "<initializing_interpolator> --> <operational>" << std::endl;
    state_ = ControllerState::Operational;

    next_print_time_ = eval_time_at_last_callback_;
    print_period_ = ros::Duration(1.);

  } else if (state_ == ControllerState::Operational) {
    AddPointToInterpolator(command_msg);

    auto interp_time = (eval_time_at_last_callback_ - eval_time_at_interpolator_start_).toSec();
    auto command_time = (command_msg.t - command_time_at_interpolator_start_).toSec();
    auto time_error = command_time - (interp_time + time_offset_);
    auto now = (ros::Time::now() - eval_time_at_interpolator_start_).toSec();

    cortex_control::CortexCommandAck command_ack;
    command_ack.cortex_command_time = command_msg.t;
    command_ack.cortex_command_id = command_msg.id;
    command_ack.time_offset = ros::Duration(-time_error);
    cortex_command_time_pub_.publish(command_ack);

    if (eval_time_at_last_callback_ >= next_print_time_) {
      std::cout << std::setprecision(10) << "[stream interpolator (" << time_offset_ << ")] "
                << "interp time: " << interp_time << ", now: " << now
                << ", command time: " << command_time << ", interp - command diff: " << -time_error
                << std::endl;

      next_print_time_ += print_period_;
    }
  }
}

void CommandStreamInterpolator::ResetInterpolator() {
  start_blending_ = true;
  if (use_smoothing_interpolator_) {
    // Auto smoothing quartic interpolation. This version always interpolates between the latest
    // evaluated (q, qd, qdd) and the incoming (q_target, qd_target).
    interp_ = std::make_shared<
        cortex::math::SmoothingIncrementalInterpolator<cortex::math::CubicPositionInterpolatorXd>>();
  } else {
    // Basic quintic interpolation.
    interp_ = std::make_shared<cortex::math::IncrementalInterpolator>();
  }
}

}  // namespace control
}  // namespace cortex
