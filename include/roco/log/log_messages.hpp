/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file     log_messages.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once

#include "roco/log/log_backend.hpp"

namespace roco {
namespace log {



#define ROCO_FATAL(...) \
    { \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << roco::log::colorFatal << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \

#define ROCO_FATAL_STREAM(message) \
    { \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << roco::log::colorFatal << message << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \

#define ROCO_ERROR(...) \
    { \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << roco::log::colorError << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_fatal>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \

#define ROCO_ERROR_STREAM(message) \
    { \
      std::stringstream roco_assert_stringstream;             \
      roco_assert_stringstream << roco::log::colorError << message << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_error>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \

#define ROCO_WARN(...) ROCO_LOG(::roco::log::levels::Warn, __VA_ARGS__)
#define ROCO_WARN_FP(...) ROCO_LOG_FP(::roco::log::levels::Warn, __VA_ARGS__)
#define ROCO_WARN_STREAM(message) ROCO_LOG_STREAM(::roco::log::levels::Warn, message)
#define ROCO_WARN_STREAM_FP(message) ROCO_LOG_STREAM_FP(::roco::log::levels::Warn, message)
#define ROCO_WARN_THROTTLE(rate, ...) ROCO_LOG_THROTTLE(rate, ::roco::log::levels::Warn, __VA_ARGS__)
#define ROCO_WARN_THROTTLE_STREAM(rate, message) ROCO_LOG_THROTTLE_STREAM(rate, ::roco::log::levels::Warn, message)

#define ROCO_INFO( ...) ROCO_LOG(::roco::log::levels::Info, __VA_ARGS__)
#define ROCO_INFO_FP(...) ROCO_LOG_FP(::roco::log::levels::Info, __VA_ARGS__)
#define ROCO_INFO_STREAM(message) ROCO_LOG_STREAM(::roco::log::levels::Info, message)
#define ROCO_INFO_STREAM_FP(message) ROCO_LOG_STREAM_FP(::roco::log::levels::Info, message)
#define ROCO_INFO_THROTTLE(rate, ...) ROCO_LOG_THROTTLE(rate, ::roco::log::levels::Info, __VA_ARGS__)
#define ROCO_INFO_THROTTLE_STREAM(rate, message) ROCO_LOG_THROTTLE_STREAM(rate, ::roco::log::levels::Info, message)

#ifndef NDEBUG
#define ROCO_DEBUG(...) ROCO_LOG(::roco::log::levels::Debug, __VA_ARGS__)
#define ROCO_DEBUG_FP(...) ROCO_LOG_FP(::roco::log::levels::Debug, __VA_ARGS__)
#define ROCO_DEBUG_STREAM(message) ROCO_LOG_STREAM(::roco::log::levels::Debug, message)
#define ROCO_DEBUG_STREAM_FP(message) ROCO_LOG_STREAM_FP(::roco::log::levels::Debug, message)
#define ROCO_DEBUG_THROTTLE(rate, ...) ROCO_LOG_THROTTLE(rate, ::roco::log::levels::Debug, __VA_ARGS__)
#else
#define ROCO_DEBUG(...)
#define ROCO_DEBUG_FP(...)
#define ROCO_DEBUG_STREAM(message)
#define ROCO_DEBUG_STREAM_FP(message)
#define ROCO_DEBUG_THROTTLE(rate, ...)
#endif

} // namespace log
} // namespace roco
