/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#pragma once

#include <memory>

#include <yaml-cpp/node/node.h>

#include "cortex/control/command_stream_interpolator.h"

namespace cortex {
namespace control {

//! Makes and initializes a command stream interpolator from the specified YAML config. One still needs
//! to call Start() on the returned object to start the streaming interpolation.
std::shared_ptr<CommandStreamInterpolator> LoadCommandStreamInterpolatorFromYaml(
    const YAML::Node& command_stream_interpolator_config, bool verbose = false);

}  // namespace control
}  // namespace cortex
