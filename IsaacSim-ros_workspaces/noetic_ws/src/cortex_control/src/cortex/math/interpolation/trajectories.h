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
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/assert.h>

#include "cortex/math/interpolation/pos_vel_acc.h"

namespace cortex {
namespace math {

// Represents a multidimensional trajectory as a collection of 1D trajectories.
template <class traj1d_t, class vec_t>
class MultiDimTraj {
 public:
  typedef vec_t VectorXx;

  MultiDimTraj() {}
  explicit MultiDimTraj(const std::vector<traj1d_t>& trajectories) : trajectories_(trajectories) {}

  bool Eval(double t, PosVelAcc<vec_t>& ret, std::string* error_str) const;

  // This verion asserts on error.
  PosVelAccXd Eval(double t) const {
    PosVelAccXd ret;
    std::string error_str;
    ROS_ASSERT_MSG(Eval(t, ret, &error_str), "%s", error_str.c_str());
    return ret;
  }

  int dim() const { return trajectories_.size(); }

 protected:
  std::vector<traj1d_t> trajectories_;
};

// Creates a vector of 1D interpolators for each dimension of the given
// PosVelAcc end-point objects. If validate_interpolation_evals is true, the
// resulting interpolators will validate that the query points are between 0
// and 1.
template <class interp1d_t, class vec_t>
std::vector<interp1d_t> MakeDimInterps(const PosVelAcc<vec_t>& p0,
                                       const PosVelAcc<vec_t>& p1,
                                       bool validate_interpolation_evals) {
  ROS_ASSERT(p0.dim() == p1.dim());
  std::vector<interp1d_t> trajectories;
  for (int i = 0; i < p0.dim(); ++i) {
    trajectories.push_back(interp1d_t(
        PosVelAcc1d::Slice(p0, i), PosVelAcc1d::Slice(p1, i), validate_interpolation_evals));
  }
  return trajectories;
}

// Represents a multi-dimensional interpolator interpolating between a pair of
// PosVelAcc points.
template <class interp1d_t, class vec_t>
class MultiDimInterp : public MultiDimTraj<interp1d_t, vec_t> {
 public:
  typedef vec_t VectorXx;

  MultiDimInterp() {}
  MultiDimInterp(const PosVelAcc<vec_t>& p0,
                 const PosVelAcc<vec_t>& p1,
                 bool validate_interpolation_evals = false)
      : MultiDimTraj<interp1d_t, vec_t>(
            MakeDimInterps<interp1d_t, vec_t>(p0, p1, validate_interpolation_evals)) {}

 protected:
};

// Represents a trajectory whose time is scaled by some scaling factor. The
// semantics of scaling is that if the original time interval were [0,1] the
// new time interval would be [0, scalar], i.e. the original trajectory on
// [0,1] would be stretched to fit across the entire interval [0, scalar].
template <class traj_t>
class TimeScaledTraj {
 public:
  typedef typename traj_t::VectorXx VectorXx;

  TimeScaledTraj() {}
  TimeScaledTraj(const traj_t& traj, double scalar) : traj_(traj), scalar_(scalar) {}

  VectorXx operator()(double t) const { return Eval(t).x; }

  PosVelAcc<typename traj_t::VectorXx> Eval(double t) const {
    return traj_.Eval(t).Unscale(scalar_);
  }

  bool Eval(double t, PosVelAcc<VectorXx>& ret, std::string* error_str = nullptr) const {
    PosVelAcc<VectorXx> scaled_ret;
    if (!traj_.Eval(t, scaled_ret, error_str)) {
      return false;
    }
    ret = scaled_ret.Unscale(scalar_);
    return true;
  }

  double scalar() const { return scalar_; }

 protected:
  traj_t traj_;
  double scalar_;
};
template <class traj_t>
TimeScaledTraj<traj_t> TimeScaleTraj(const traj_t& traj, double scalar) {
  return TimeScaledTraj<traj_t>(traj, scalar);
}

// traj_t should have an evaluation operator:
//
//   vec_t operator()(double t) const
//
// This function performs finite-differencing to find the velocity.
// traj_t should also have a type vec_t:
//
//   typename traj_t::vec_t
//
template <class traj_t>
typename traj_t::VectorXx CentralFdVel(const traj_t& traj, double t, double dt = 1e-5) {
  auto x_up = traj(t + dt / 2);
  auto x_down = traj(t - dt / 2);
  return (x_up - x_down) / dt;
}

template <class traj_t>
typename traj_t::VectorXx FdAcc(const traj_t& traj, double t, double dt = 1e-5) {
  auto x = traj(t);
  auto x_up = traj(t + dt / 2);
  auto x_down = traj(t - dt / 2);
  return (x_up + x_down - 2 * x) / (dt * dt / 4);
}

// Converts a trajectory into a velocity trajectory using finite-differencing.
template <class traj_t>
class FdVelTraj {
 public:
  typedef typename traj_t::VectorXx VectorXx;

  explicit FdVelTraj(const traj_t& traj, double dt = 1e-5) : traj_(traj), dt_(dt) {}

  VectorXx operator()(double t) const { return CentralFdVel(traj_, t, dt_); }

 protected:
  traj_t traj_;
  double dt_;
};

template <class traj_t>
FdVelTraj<traj_t> ToFdVelTraj(const traj_t& traj) {
  return FdVelTraj<traj_t>(traj);
}

// Converts a trajectory into an acceleration trajectory using
// finite-differencing.
template <class traj_t>
class FdAccTraj {
 public:
  typedef typename traj_t::VectorXx VectorXx;

  explicit FdAccTraj(const traj_t& traj, double dt = 1e-5) : traj_(traj), dt_(dt) {}

  VectorXx operator()(double t) const { return FdAcc(traj_, t, dt_); }

 protected:
  traj_t traj_;
  double dt_;
};

template <class traj_t>
FdAccTraj<traj_t> ToFdAccTraj(const traj_t& traj) {
  return FdAccTraj<traj_t>(traj);
}

// Represents f(t) = c1 * sin(c2 * (t - t0)) + c3
//
// Derivatives:
//   f' = c1 * c2 * cos(c2 * (t - t0))
//   f'' = -c1 * c2^2 * sin(c2 * (t - t0))
class SinusoidalTraj {
 public:
  typedef double VectorXx;

  SinusoidalTraj(double c1, double c2, double c3, double t0) : c1_(c1), c2_(c2), c3_(c3), t0_(t0) {}

  PosVelAcc1d Eval(double t) const {
    std::string error_str;

    PosVelAcc1d ret;
    ROS_ASSERT_MSG(Eval(t, ret, &error_str), "%s", error_str.c_str());

    return ret;
  }

  bool Eval(double t, PosVelAcc1d& ret, std::string* error_str = nullptr) const {
    // Suppress warnings that "error_str" is never written to.
    (void)error_str;

    auto t_affine = c2_ * (t - t0_);
    auto x = c1_ * sin(t_affine) + c3_;
    auto xd = c1_ * c2_ * cos(t_affine);
    auto xdd = -c1_ * c2_ * c2_ * sin(t_affine);
    ret = PosVelAcc1d(x, xd, xdd);
    return true;
  }

  double operator()(double t) const { return Eval(t).x; }

 protected:
  double c1_, c2_, c3_, t0_;
};

//==============================================================================
// Template implementations
//==============================================================================

template <class traj1d_t, class vec_t>
bool MultiDimTraj<traj1d_t, vec_t>::Eval(double t,
                                         PosVelAcc<vec_t>& ret,
                                         std::string* error_str) const {
  std::vector<PosVelAcc1d> dim_evals(dim());
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    if (!trajectories_[i].Eval(t, dim_evals[i], error_str)) {
      return false;
    }
  }
  ret = PosVelAcc<vec_t>::Join(dim_evals);
  return true;
}

}  // namespace math
}  // namespace cortex
