/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/math/interpolation/incremental_interpolator.h"

#include <fstream>
#include <string>

#include "cortex/math/interpolation/quintic_interpolator.h"

namespace cortex {
namespace math {

IncrementalInterpolator::IncrementalInterpolator(bool prune_history,
                                                 bool validate_interpolation_evals)
    : is_first_(true),
      validate_interpolation_evals_(validate_interpolation_evals),
      prune_history_(prune_history) {}

bool IncrementalInterpolator::AddPt(double t, const PosVelAccXd& p, std::string* error_str) {
  if (is_first_) {
    prev_add_t_ = t;
    prev_add_p_ = p;
    is_first_ = false;
    return true;
  }

  if (t <= prev_add_t_) {
    if (error_str) {
      std::stringstream ss;
      ss << "Add times nonmonotonic -- t = " << t << " vs prev t = " << prev_add_t_;
      *error_str += ss.str();
    }
    return false;
  }

  segment_interpolators_.push_back(
      TimeScaledInterpolatorXd(prev_add_t_, prev_add_p_, t, p, validate_interpolation_evals_));
  prev_add_t_ = t;
  prev_add_p_ = p;

  return true;
}

bool IncrementalInterpolator::Eval(double t, PosVelAccXd& ret, std::string* error_str) const {
  if (segment_interpolators_.size() == 0) {
    if (error_str) {
      *error_str += "No interpolators found.";
    }
    return false;
  }

  auto earliest_time = segment_interpolators_.front().t0();
  auto latest_time = segment_interpolators_.back().t1();
  if (validate_interpolation_evals_ && t < earliest_time) {
    if (error_str) {
      std::stringstream ss;
      ss << "Nonmonotonic evals -- t = " << t
         << ", earliest time segment starts with t0 = " << earliest_time;
      *error_str += ss.str();
    }
    return false;
  }
  if (validate_interpolation_evals_ && t > latest_time) {
    if (error_str) {
      std::stringstream ss;
      ss << "Future eval (overflow) -- t = " << t
         << ", latest time segment ends with t1 = " << latest_time;
      *error_str += ss.str();
    }
    return false;
  }

  // Find the first segment whose upper time bound is greater than the curren
  // time. Since the segments are contiguous and monotonically increasing, we're
  // guaranteed that t \in [t0, t1] of this segment.
  TimeScaledInterpolatorXd* active_interpolator = nullptr;
  for (auto it = segment_interpolators_.begin(); it != segment_interpolators_.end();) {
    if (t <= it->t1()) {
      active_interpolator = &(*it);
      break;
    } else {
      if (prune_history_) {
        it = segment_interpolators_.erase(it);
      } else {
        ++it;
      }
    }
  }
  if (!active_interpolator && !validate_interpolation_evals_) {
    active_interpolator = &segment_interpolators_.back();
  }
  if (active_interpolator) {
    return active_interpolator->Eval(t, ret, error_str);
  } else {
    if (error_str) {
      std::stringstream ss;
      ss << "Eval time in the future -- t = " << t << " vs latest segment time = " << latest_time;
      *error_str += ss.str();
    }
    return false;
  }
}

bool IncrementalInterpolator::IsReady(double t) const {
  return (segment_interpolators_.size() > 0) && (t >= segment_interpolators_.front().t0());
}

}  // namespace math
}  // namespace cortex
