/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/math/interpolation/cubic_position_interpolator.h"

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace cortex {
namespace math {

// Returns true iff t \in [0,1].
inline bool InZeroOne(double t) { return 0 <= t && t <= 1; }

// clang-format off
#define CUBIC_POSITION_INTERP_MATRIX \
    0, 0, 0, 1, \
    0, 0, 1, 0, \
    0, 2, 0, 0, \
    1, 1, 1, 1
// clang-format on

CubicPositionInterpolator1d::CubicPositionInterpolator1d(const PosVelAcc1d& p0,
                                                         const PosVelAcc1d& p1,
                                                         bool validate_interpolation_evals)
    : validate_interpolation_evals_(validate_interpolation_evals),
      A_((Eigen::MatrixXd(4, 4) << CUBIC_POSITION_INTERP_MATRIX).finished()),
      b_((Eigen::VectorXd(4) << p0.x, p0.xd, p0.xdd, p1.x).finished()),
      coeffs_(A_.colPivHouseholderQr().solve(b_)) {}

bool CubicPositionInterpolator1d::Eval(double t, PosVelAcc1d& ret, std::string* error_str) const {
  if (validate_interpolation_evals_ && !InZeroOne(t)) {
    std::stringstream ss;
    ss << "t not in [0,1] (t = " << t << "). ";
    if (error_str) {
      *error_str += ss.str();
    }
    return false;
  }
  auto a3 = coeffs_[0];
  auto a2 = coeffs_[1];
  auto a1 = coeffs_[2];
  auto a0 = coeffs_[3];

  std::vector<double> t_powers(4, 1);
  for (size_t i = 1; i < t_powers.size(); ++i) {
    t_powers[i] = t * t_powers[i - 1];
  }

  auto x = a3 * t_powers[3] + a2 * t_powers[2] + a1 * t_powers[1] + a0;
  auto xd = 3. * a3 * t_powers[2] + 2. * a2 * t_powers[1] + a1;
  auto xdd = 6. * a3 * t_powers[1] + 2. * a2;

  ret = PosVelAcc1d(x, xd, xdd);
  return true;
}

}  // namespace math
}  // namespace cortex
