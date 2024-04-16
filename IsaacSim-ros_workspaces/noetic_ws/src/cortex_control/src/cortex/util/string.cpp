/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/util/string.h"

#include <sstream>

namespace cortex {
namespace util {

std::vector<std::string> Split(const std::string& str, char delimiter){
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream token_stream(str);
  while (std::getline(token_stream, token, delimiter)) {
    if (token.size() > 0) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

std::string Join(const std::vector<std::string>& tokens, char delimiter, size_t pos) {
  std::stringstream ss;
  for (auto i = pos; i < tokens.size(); ++i) {
    if (i > pos) ss << delimiter;
    ss << tokens[i];
  }
  return ss.str();
}

}  // namespace util
}  // namespace cortex
