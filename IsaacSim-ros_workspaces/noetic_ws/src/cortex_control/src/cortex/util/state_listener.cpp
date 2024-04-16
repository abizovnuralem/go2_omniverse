/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/util/state_listener.h"

#include <chrono>
#include <csignal>
#include <thread>

#include <ros/ros.h>

namespace cortex {
namespace util {

// std::atomic_bool StateListener::interruped_(false);

StateListener::StateListener() {
  // std::signal(SIGINT, &StateListener::signal_handler);
}

void StateListener::WaitForReady(double poll_hz) const {
  // This is an alternative and ros free implementation of the thread SIGINT
  // signal handling

  // auto sleep_duration = std::chrono::duration<double>(1. / poll_hz);
  // while (!interruped_.load() && !IsReady()) {
  //   std::this_thread::sleep_for(sleep_duration);
  // }

  ros::Rate rate(poll_hz);
  while (ros::ok() && !IsReady()) {
    rate.sleep();
  }
}

// void StateListener::signal_handler(int signal) { interruped_.store(true); }

}  // namespace util
}  // namespace cortex
