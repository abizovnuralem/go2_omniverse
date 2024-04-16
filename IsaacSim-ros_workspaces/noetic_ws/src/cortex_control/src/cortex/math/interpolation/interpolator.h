/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

// Pure virtual base class interface for an interpolator.

#pragma once

#include <iostream>
#include <list>
#include <sstream>
#include <string>

#include <ros/assert.h>

#include "cortex/math/interpolation/pos_vel_acc.h"
#include "cortex/math/interpolation/time_scaled_interpolator.h"

namespace cortex {
namespace math {

// Represents a generic interpolator interface giving an API of the form:
//
// 1. Add p = (q, qd, qdd) point at time t:
//
//    interp.AddPt(t, p);
//
// 2. Evaluate at a given time t:
//
//    auto p = interp.Eval(t);
//    auto p = interp(t);
//
// Deriving classes need to implement the pure virtual functions
//
//    AddPt()   and   Eval()
//
// Deriving classes might add additional restrictions, such as monotonicity of add
// times t (IncrementalInterpolator).
template <class vec_t>
class Interpolator {
 public:
  typedef vec_t VectorXx;

  virtual bool AddPt(double t, const PosVelAcc<VectorXx>& p, std::string* error_str = nullptr) = 0;

  virtual bool Eval(double t, PosVelAcc<VectorXx>& ret, std::string* error_str) const = 0;

  // Asserting version.
  PosVelAccXd Eval(double t) const {
    std::string error_str;
    PosVelAccXd p;
    ROS_ASSERT_MSG(Eval(t, p, &error_str), "%s", error_str.c_str());
    return p;
  }

  Eigen::VectorXd operator()(double t) const { return Eval(t).x; }
};

}  // namespace math
}  // namespace cortex
