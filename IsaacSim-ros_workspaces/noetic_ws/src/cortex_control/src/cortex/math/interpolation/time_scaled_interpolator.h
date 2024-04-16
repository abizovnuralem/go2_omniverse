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

#include <Eigen/Core>
#include <ros/assert.h>

#include "cortex/math/interpolation/pos_vel_acc.h"
#include "cortex/math/interpolation/trajectories.h"

namespace cortex {
namespace math {

// Represents a quintic interpolator interpolating between two end points
// at specific times. If validate_interpolation_evals is true, valid evals
// are only those within the time range of the two end points.
template <class traj_t>
class TimeScaledInterpolator {
 public:
  typedef typename TimeScaledTraj<traj_t>::VectorXx VectorXx;

  TimeScaledInterpolator() {}
  TimeScaledInterpolator(double t0,
                         const PosVelAcc<VectorXx>& p0,
                         double t1,
                         const PosVelAcc<VectorXx>& p1,
                         bool validate_interpolation_evals = false)
      : t0_(t0),
        p0_(p0),
        t1_(t1),
        p1_(p1),
        time_range_(t1 - t0),
        scaled_traj_(
            traj_t(p0.Scale(time_range_), p1.Scale(time_range_), validate_interpolation_evals),
            time_range_),
        validate_interpolation_evals_(validate_interpolation_evals) {}

  bool Eval(double t, PosVelAcc<VectorXx>& ret, std::string* error_str = nullptr) const {
    if (validate_interpolation_evals_ && !(t0_ <= t && t <= t1_)) {
      if (error_str) {
        std::stringstream ss;
        ss << "t = " << t << " outside valid range [" << t0_ << ", " << t1_ << "]";
        *error_str += ss.str();
      }
      return false;
    }
    return scaled_traj_.Eval((t - t0_) / time_range_, ret, error_str);
  }
  PosVelAcc<VectorXx> Eval(double t) const {
    std::string error_str;
    PosVelAcc<VectorXx> ret;
    ROS_ASSERT_MSG(scaled_traj_.Eval((t - t0_) / time_range_, ret, &error_str), "%s",
                   error_str.c_str());
    return ret;
  }

  // Performs a time shifted eval since the underlying trajectory starts at t0_
  VectorXx operator()(double t) const { return Eval(t).x; }

  double t0() const { return t0_; }
  const PosVelAcc<VectorXx>& p0() const { return p0_; }
  double t1() const { return t1_; }
  const PosVelAcc<VectorXx>& p1() const { return p1_; }

 protected:
  double t0_;
  PosVelAcc<VectorXx> p0_;
  double t1_;
  PosVelAcc<VectorXx> p1_;

  double time_range_;
  TimeScaledTraj<traj_t> scaled_traj_;
  bool validate_interpolation_evals_;
};

}  // namespace math
}  // namespace cortex
