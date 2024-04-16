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

#include <cstdint>

#include <Eigen/Core>

namespace cortex {
namespace util {

struct StampedState {
  double time;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd u;

  int dim() const { return q.size(); }

  StampedState() = default;
  StampedState(uint32_t num_dim);
  StampedState(double time, const Eigen::VectorXd &q, const Eigen::VectorXd &qd);
  virtual ~StampedState() = default;

  bool HasU() const;
};

}  // namespace util
}  // namespace cortex
