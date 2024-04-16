/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

#include <atomic>

#include <Eigen/Core>

#include "cortex/util/stamped_state.h"

namespace cortex {
namespace util {

/**
 * \brief Abstract state listener.
 */
class StateListener {
 public:
  /**
   * \brief Creates a StateListener.
   */
  StateListener();

  /**
   * \brief Default virtual destructor.
   */
  virtual ~StateListener() = default;

  /**
   * \brief Returns the latest state.
   */
  virtual StampedState State() const = 0;

  /**
   * \brief Returns true if the state is available.
   */
  virtual bool IsReady() const = 0;

  /**
   * \brief Blocking call to wait until the state is available.
   */
  virtual void WaitForReady(double poll_hz = 100) const;

 private:
  // This is an alternative and ros free implementation of the thread SIGINT
  // signal handling
  // static void signal_handler(int signal);
  // static std::atomic_bool interruped_;
};

}  // namespace util
}  // namespace cortex
