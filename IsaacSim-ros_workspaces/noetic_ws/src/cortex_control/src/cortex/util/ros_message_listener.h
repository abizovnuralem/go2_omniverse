/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <vector>

#include <ros/ros.h>

namespace cortex {
namespace util {

// Generic message listener that saves off the latest message and makes it available atomically.
//
// Includes flag accessor is_available() saying whether the first message has been received.
// Thereafter, it always reports the last received message through GetLatestMessage(). There is no
// timeout mechanism on these messages, so once is_available() returns true for the first time, it
// will be true for every call after that.
template <class msg_t>
class RosMessageListener {
 public:
  RosMessageListener(const std::string& topic, int queue_size) {
    is_available_ = false;
    ros::NodeHandle node_handle;
    sub_ = node_handle.subscribe(topic, queue_size, &RosMessageListener<msg_t>::Callback, this);
  }

  void Callback(const msg_t& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    msg_ = msg;
    is_available_ = true;

    for (auto& f : callbacks_) {
      f(msg_);
    }
  }

  bool is_available() const { return is_available_; }

  msg_t GetLatestMessage() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return msg_;
  }

  void RegisterCallback(const std::function<void(const msg_t&)>& f) { callbacks_.push_back(f); }

 protected:
  mutable std::mutex mutex_;
  ros::Subscriber sub_;
  std::atomic_bool is_available_;

  msg_t msg_;

  std::vector<std::function<void(const msg_t&)>> callbacks_;
};

}  // namespace util
}  // namespace cortex
