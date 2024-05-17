/*
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <ros/ros.h>

#include "cortex/util/state_listener.h"

namespace cortex {
namespace util {

bool StampedState::HasU() const { return u.size() > 0; }

StampedState::StampedState(uint32_t num_dim)
    : time(0.),
      q(Eigen::VectorXd::Zero(num_dim)),
      qd(Eigen::VectorXd::Zero(num_dim)),
      u(Eigen::VectorXd::Zero(num_dim)) {}

StampedState::StampedState(double time, const Eigen::VectorXd &q, const Eigen::VectorXd &qd)
    : time(time), q(q), qd(qd) {}

}  // namespace util
}  // namespace cortex
