/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

#include <Eigen/Core>

namespace cortex {
namespace math {

// Represents a vector s = (x, xd) \in \R^{2d} where d is the space dim.
class State {
 public:
  State() = delete;
  explicit State(int n);  // Initialize to the zero state (0,0)\in\R^n X \R^n.
  State(const Eigen::VectorXd &x, const Eigen::VectorXd &xd);

  Eigen::Ref<Eigen::VectorXd> pos() { return state.head(dim()); }
  Eigen::Ref<Eigen::VectorXd> vel() { return state.tail(dim()); }
  Eigen::Ref<Eigen::VectorXd> vector() { return state; }
  Eigen::Ref<const Eigen::VectorXd> pos() const { return state.head(dim()); }
  Eigen::Ref<const Eigen::VectorXd> vel() const { return state.tail(dim()); }
  Eigen::Ref<const Eigen::VectorXd> vector() const { return state; }

  int dim() const { return state.size() / 2; }

  // Returns one integration step forward.
  //
  // Equations:
  //   x_next = x + dt xd
  //   xd_next = xd + dt xdd
  State Step(double dt, const Eigen::VectorXd &xdd) {
    return State(pos() + dt * vel(), vel() + dt * xdd);
  }

 private:
  Eigen::VectorXd state;
};

}  // namespace math
}  // namespace cortex
