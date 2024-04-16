/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

// Simple and generic smoothing incremental interpolator that creates each new
// polynomial segment between the latest evaluated point (the point sent to
// control) and the incoming point. This adds a level of robustness to noise
// governed by the size of the eval shift window.

#pragma once

#include <iostream>
#include <list>
#include <sstream>
#include <string>

#include "cortex/math/interpolation/interpolator.h"
#include "cortex/math/interpolation/pos_vel_acc.h"
#include "cortex/math/interpolation/time_scaled_interpolator.h"

namespace cortex {
namespace math {

template <class interp_t>
class SmoothingIncrementalInterpolator : public Interpolator<typename interp_t::VectorXx> {
 public:
  SmoothingIncrementalInterpolator() : is_first_(true), is_ready_(false) {}

  bool AddPt(double t,
             const PosVelAcc<typename interp_t::VectorXx>& p,
             std::string* error_str = nullptr) override {
    if (is_first_) {
      prev_eval_t_ = t;
      prev_eval_p_ = p;
      is_first_ = false;
      return true;
    }
    is_ready_ = true;

    if (t <= prev_eval_t_) {
      if (error_str) {
        std::stringstream ss;
        ss << "Add time must be beyond the last eval time = " << t
           << " vs last eval t = " << prev_eval_t_;
        *error_str += ss.str();
      }
      return false;
    }

    interpolator_ = TimeScaledInterpolator<interp_t>(prev_eval_t_, prev_eval_p_, t, p);

    return true;
  }

  // Note: only adds to the error string if there's an error. Typically string
  // operations aren't real time safe, but in this case we'd be bailing out.
  bool Eval(double t,
            PosVelAcc<typename interp_t::VectorXx>& ret,
            std::string* error_str) const override {
    if (!IsReady(t)) {
      if (error_str) {
        *error_str +=
            "Smoothing increment interpolator not ready. Must see at least two "
            "points before evaluating.";
      }
      return false;
    }
    if (t < interpolator_.t0()) {
      if (error_str) {
        std::stringstream ss;
        ss << "Nonmonotonic evals -- t = " << t << ", last eval was at " << interpolator_.t0();
        *error_str += ss.str();
      }
      return false;
    }
    if (t > interpolator_.t1()) {
      // TODO(roflaherty): Convert this over to a version that extrapolates with zero
      // acceleration. Include a jitter buffer (only extrapolate so far).
      //
      // For now, though, this is unsupported and it just errors.
      if (error_str) {
        std::stringstream ss;
        ss << "Future eval requested. Currently unsupported. Expects eval "
           << "monotonicity -- t = " << t << ", last eval time = " << interpolator_.t1();
        *error_str += ss.str();
      }
      return false;
    }

    if (!interpolator_.Eval(t, ret, error_str)) {
      return false;
    }
    prev_eval_t_ = t;
    prev_eval_p_ = ret;
    return true;
  }

  using Interpolator<typename interp_t::VectorXx>::Eval;

  // Returns true iff the interpolator was created as least enough time in the
  // past so the shifted evaluation time falls within the valid range of the
  // interpolator.
  //
  // Note that once the interpolator is ready (has return ready once), since
  // new interpolators are always created to be lower bounded at the shifted
  // interpolation eval time, and eval times are always monotonically
  // increasing, it will always be ready (always return true).
  bool IsReady(double t) const { return is_ready_ && (t >= interpolator_.t0()); }

 protected:
  TimeScaledInterpolator<interp_t> interpolator_;

  bool is_first_;
  bool is_ready_;
  mutable double prev_eval_t_;
  mutable PosVelAccXd prev_eval_p_;
};

}  // namespace math
}  // namespace cortex
