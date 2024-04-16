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

#include <string>
#include <vector>

namespace cortex {
namespace util {

//! Split the specified string `str` into a set of strings delimited by the `delimiter` character.
//! If the delimiter is not found, the entire string is returned as a single token.  The returned
//! vector always contains, in union, the set of all characters in the string that aren't
//! delimiters.
std::vector<std::string> Split(const std::string& str, char delimiter);

//! Join the tokens together separated by the specified `delimiter` character.  Start with token
//! `pos`.  By default, `pos` is zero, so all tokens are included.
std::string Join(const std::vector<std::string>& tokens, char delimiter, size_t pos = 0);

}  // namespace util
}  // namespace cortex
