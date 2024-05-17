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

#include <Eigen/Core>

#include "cortex/util/state_listener.h"

namespace cortex {
namespace util {

/**
 * \brief This is a very simple state listener that just reports its set state.
 */
class SetStateListener : public StateListener {
 public:
  SetStateListener() : is_set_(false) {}
  StampedState State() const override { return state_; }
  bool IsReady() const override { return is_set_; }
  void set_stamped_state(const StampedState &state) { state_ = state; }

 protected:
  bool is_set_;
  StampedState state_;
};

}  // namespace util
}  // namespace cortex
