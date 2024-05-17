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
#include "cortex/math/interpolation/time_scaled_interpolator.h"
#include "cortex/math/interpolation/trajectories.h"

namespace cortex {
namespace math {

// One-dimensional quintic interpolating polynomial.
class QuinticInterpolator1d {
 public:
  typedef double VectorXx;

  QuinticInterpolator1d() {}

  // Creates a quintic spline that interpolates between p0 and p1 at t = 0 and
  // 1, respectively.
  QuinticInterpolator1d(const PosVelAcc1d& p0,
                        const PosVelAcc1d& p1,
                        bool validate_interpolation_evals = false);

  // Evaluate the polynomial at t. If validate_interpolating_evals is set to
  // true, enforces that the evaluations are only interpolating, i.e. t is in
  // [0, 1]; fails if not. The interpolated value is returned in the ret return
  // parameter. On failure, returns false and sets the error string if it's
  // provided.
  bool Eval(double t, PosVelAcc1d& ret, std::string* error_str = nullptr) const;

  // This verion asserts on error.
  PosVelAcc1d Eval(double t) const {
    PosVelAcc1d ret;
    std::string error_str;
    ROS_ASSERT_MSG(Eval(t, ret, &error_str), "%s", error_str.c_str());
    return ret;
  }

  double operator()(double t) const {
    auto p = Eval(t);
    return p.x;
  }

  // Accessor.
  const Eigen::VectorXd& coeffs() const { return coeffs_; }

 protected:
  bool validate_interpolation_evals_;
  const Eigen::MatrixXd A_;
  const Eigen::VectorXd b_;
  const Eigen::VectorXd coeffs_;
};

template <class vec_t>
MultiDimInterp<QuinticInterpolator1d, vec_t> QuinticInterpolator(
    const PosVelAcc<vec_t>& p0,
    const PosVelAcc<vec_t>& p1,
    bool validate_interpolation_evals = false) {
  return MultiDimInterp<QuinticInterpolator1d, vec_t>(p0, p1, validate_interpolation_evals);
}

typedef MultiDimInterp<QuinticInterpolator1d, Eigen::VectorXd> QuinticInterpolatorXd;
typedef TimeScaledInterpolator<QuinticInterpolatorXd> TimeScaledInterpolatorXd;

}  // namespace math
}  // namespace cortex
