/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/control/builders.h"
#include "cortex/util/yaml.h"

#include <ros/time.h>

namespace cortex {
namespace control {

std::shared_ptr<CommandStreamInterpolator> LoadCommandStreamInterpolatorFromYaml(
    const YAML::Node& command_stream_interpolator_config, bool verbose) {
  // Extract params from yaml config.
  auto params = util::GetFieldOrDie(command_stream_interpolator_config, "params");
  auto interpolation_delay = util::GetOrDie<double>(params, "interpolation_delay");
  auto use_smoothing_interpolator = util::GetOrDie<bool>(params, "use_smoothing_interpolator");
  auto blending_duration = util::GetOrDie<double>(params, "blending_duration");
  auto backend_timeout = util::GetOrDie<double>(params, "backend_timeout");

  // Extract ROS topics from yaml config.
  auto ros_topics = util::GetFieldOrDie(command_stream_interpolator_config, "ros_topics");
  auto command_topics = util::GetFieldOrDie(ros_topics, "rmpflow_commands");
  auto rmpflow_command_topic = util::GetOrDie<std::string>(command_topics, "command");
  auto rmpflow_command_ack_topic = util::GetOrDie<std::string>(command_topics, "ack");
  auto rmpflow_command_suppress_topic = util::GetOrDie<std::string>(command_topics, "suppress");
  auto rmpflow_command_interpolated_topic = util::GetOrDie<std::string>(command_topics,
                                                                        "interpolated");

  if (verbose) {
    std::cout << "RMPflow backend config:" << std::endl;
    std::cout << " params:" << std::endl;
    std::cout << "  interpolation delay: " << interpolation_delay << std::endl;
    std::cout << "  use smoothing interpolator: " << use_smoothing_interpolator << std::endl;
    std::cout << "  blending duration: " << blending_duration << std::endl;
    std::cout << "  backend timeout: " << backend_timeout << std::endl;
    std::cout << " ros_topics:" << std::endl;
    std::cout << "  rmpflow_commands:" << std::endl;
    std::cout << "   command: " << rmpflow_command_topic << std::endl;
    std::cout << "   ack: " << rmpflow_command_ack_topic << std::endl;
    std::cout << "   suppress: " << rmpflow_command_suppress_topic << std::endl;
    std::cout << "   interpolated: " << rmpflow_command_interpolated_topic << std::endl;
  }

  auto stream_interpolator = std::make_shared<cortex::control::CommandStreamInterpolator>();
  stream_interpolator->Init(ros::Duration(interpolation_delay),
                            use_smoothing_interpolator,
                            rmpflow_command_topic,
                            rmpflow_command_ack_topic,
                            rmpflow_command_suppress_topic,
                            rmpflow_command_interpolated_topic,
                            ros::Duration(blending_duration),
                            backend_timeout);
  return stream_interpolator;
}

}  // namespace control
}  // namespace cortex
