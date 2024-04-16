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

#include "cortex/math/interpolation/interpolator.h"
#include "cortex/math/interpolation/pos_vel_acc.h"
#include "cortex/math/interpolation/quintic_interpolator.h"

// Note: an incremental interpolator is one that leverages monotonicity
// assumptions on the evaluation times to continually grow the interpolator head
// while removing stale segments from the tail.

namespace cortex {
namespace math {

// Enables the interpolation of a sequence of (x, xd, xdd) way-points using
// quintic polynomials for each region between points. Evaluations and adding
// of new way points can be interleaved, although evaluations are expected to
// be with monotonically increasing time. There's a notion of a "delay_buffer"
// which enables points to be received and added with wall-clock time
// simultaneous with wall-clock evaluations by evaluating at a fixed time
// interval in the past. The delay buffer is the number of intervals in the past
// to set that fixed time offset to.
//
// When interpolating between (x, xd, xdd) way points at a non unity dt
// (i.e. each way point is dt seconds apart), we need to scale the xd and
// xdd by dt and dt^2, respectively, when adding them and undo that scaling
// when evaluating. Intuition: if dt is small, it's moving fast from one
// point to the next. If we then interpolate pretending that it takes a
// full second to get from one to the next, it's moving and accelerating
// much much slower, so we need to scale by dt and dt^2.
//
// This can be more rigorously derived by looking how time dilation scalars
// propagate through the derivative expressions.
class IncrementalInterpolator : public Interpolator<Eigen::VectorXd> {
 public:
  explicit IncrementalInterpolator(bool prune_history = true,
                                   bool validate_interpolation_evals = true);

  // Add a new waypoint, the time should be the current cycle time. Evals will
  // be offset into the past by delay_buffer number of intervals to that
  // incoming points can be added with the same time stamp as active
  // evaluations.
  bool AddPt(double t, const PosVelAccXd& p,
             std::string* error_str = nullptr) override;

  // Evaluates the interpolator at the given time. It uses a delay buffer to
  // offset the evaluations into the past so that points can be added at the
  // same time as evaluations and evaluations can be made after the latest
  // point safely as long as they're within the delay buffer (see description
  // above).
  //
  // This delay buffer functionality can also be implemented manually simply by
  // setting the delay_buffer to zero no construction and manually offsetting
  // evaluation points into the past.
  //
  // It's assumed the eval points are monotonically increasing. Fails if not.
  // the evaluation point is returned as ret.  Returns true if successful and
  // false otherwise.
  bool Eval(double t, PosVelAccXd& ret,
            std::string* error_str = nullptr) const override;
  using Interpolator<Eigen::VectorXd>::Eval;

  int num_intervals() const { return segment_interpolators_.size(); }

  bool IsReady(double t) const;

 protected:
  mutable std::list<TimeScaledInterpolatorXd> segment_interpolators_;

  bool is_first_;
  double prev_add_t_;
  PosVelAccXd prev_add_p_;

  bool validate_interpolation_evals_;
  bool prune_history_;
};

typedef IncrementalInterpolator SequentialQuinticInterpolator;

}  // namespace math
}  // namespace cortex
