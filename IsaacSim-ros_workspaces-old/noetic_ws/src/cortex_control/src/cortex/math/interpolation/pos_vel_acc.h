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

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <ros/assert.h>

namespace cortex {
namespace math {

struct PosVelAcc1d;

// Represents a triple of simultaneous position, velocity, and acceleration.
template <class vec_t>
struct PosVelAcc {
  vec_t x;
  vec_t xd;
  vec_t xdd;

  int dim() const { return x.size(); }
  PosVelAcc<vec_t> Scale(double dt) const { return PosVelAcc<vec_t>(x, dt * xd, (dt * dt) * xdd); }
  PosVelAcc<vec_t> Unscale(double dt) const {
    return PosVelAcc<vec_t>(x, xd / dt, xdd / (dt * dt));
  }

  PosVelAcc() {}

  // Initialize to all zeros with a particular dimensionality.
  explicit PosVelAcc(int d) {
    x = vec_t::Zero(d);
    xd = vec_t::Zero(d);
    xdd = vec_t::Zero(d);
  }

  // Initialize to specific (x, xd, xdd). Each vector much be the same
  // dimension, otherwise assert.
  PosVelAcc(const vec_t& x, const vec_t& xd, const vec_t& xdd);

  // Join a collection of one-dimensional PosVelAcc1d's into a single object of
  // this type. Aggregates the individual dimensions into vectors, x, xd, xdd.
  static PosVelAcc Join(const std::vector<PosVelAcc1d>& dims);
};

// One dimensional variant of pos, vel, acc.
struct PosVelAcc1d {
  double x;
  double xd;
  double xdd;

  PosVelAcc1d() {}
  PosVelAcc1d(double x, double xd, double xdd) : x(x), xd(xd), xdd(xdd) {}

  // Slice a multi-dimensional pos, vel, acc into a one-dimensional variant
  // containing only the specified dimension.
  template <class vec_t>
  static PosVelAcc1d Slice(const PosVelAcc<vec_t>& p, int dim) {
    return PosVelAcc1d(p.x[dim], p.xd[dim], p.xdd[dim]);
  }
};

//==============================================================================
// Template implementations
//==============================================================================

template <class vec_t>
PosVelAcc<vec_t>::PosVelAcc(const vec_t& x, const vec_t& xd, const vec_t& xdd)
    : x(x), xd(xd), xdd(xdd) {
  ROS_ASSERT(x.size() == xd.size());
  ROS_ASSERT(x.size() == xdd.size());
}

template <class vec_t>
PosVelAcc<vec_t> PosVelAcc<vec_t>::Join(const std::vector<PosVelAcc1d>& dims) {
  PosVelAcc<vec_t> p(dims.size());
  for (size_t i = 0; i < dims.size(); ++i) {
    p.x[i] = dims[i].x;
    p.xd[i] = dims[i].xd;
    p.xdd[i] = dims[i].xdd;
  }
  return p;
}

// Add specialization for VectorXd for convenience.
typedef PosVelAcc<Eigen::VectorXd> PosVelAccXd;

}  // namespace math
}  // namespace cortex


inline std::ostream& operator<<(std::ostream& os, const cortex::math::PosVelAcc1d& p) {
  os << " x = " << p.x << ", xd = " << p.xd << ", xdd = " << p.xdd;
  return os;
}

template <class vec_t>
std::ostream& operator<<(std::ostream& os, const cortex::math::PosVelAcc<vec_t>& p) {
  os << "x = " << p.x.transpose() << "\n";
  os << "xd = " << p.xd.transpose() << "\n";
  os << "xdd = " << p.xdd.transpose() << "\n";
  return os;
}
