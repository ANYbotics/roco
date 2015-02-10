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
* @file     log_messages_std.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once


#include "roco/log/log_messages_backend.hpp"
#include "roco/time/TimeStd.hpp"


namespace roco {
namespace log {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG(level, ...) \
{ \
  switch (level) { \
    case roco::log::levels::Error: \
    { \
      std::stringstream roco_assert_stringstream; \
      roco_assert_stringstream << roco::log::colorError << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_error>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \
    break; \
  case roco::log::levels::Fatal: \
    { \
      std::stringstream roco_assert_stringstream; \
      roco_assert_stringstream << roco::log::colorFatal << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      roco::common::internal::roco_throw_exception<roco::log::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
    } \
    break; \
  default: \
    { \
      std::stringstream roco_stringstream; \
      roco_stringstream << roco::log::getLogColor(level) << "[CTRL " << roco::log::getLogLevel(level)  << "] " << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      std::cout << roco_stringstream.str() << std::endl; \
    } \
    break; \
  } \
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG_STREAM(level, message) \
    { \
      switch (level) { \
      case roco::log::levels::Error: \
        { \
          std::stringstream roco_assert_stringstream;             \
          roco_assert_stringstream << roco::log::colorError << message << roco::log::getResetColor(); \
          roco::common::internal::roco_throw_exception<roco::log::roco_error>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
        } \
        break; \
      case roco::log::levels::Fatal: \
        { \
          std::stringstream roco_assert_stringstream;             \
          roco_assert_stringstream << roco::log::colorFatal << message << roco::log::getResetColor(); \
          roco::common::internal::roco_throw_exception<roco::log::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str()); \
        } \
        break; \
      default: \
        { \
          std::stringstream roco_stringstream; \
          roco_stringstream << roco::log::getLogColor(level) << "[CTRL " << roco::log::getLogLevel(level)  << "] " << message << roco::log::getResetColor(); \
          std::cout << roco_stringstream.str() << std::endl; \
        } \
        break; \
      } \
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG_FP(level, ...) \
    { \
      std::stringstream roco_stringstream; \
      roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
      roco_stringstream << roco::log::getLogColor(level) << "[CTRL " << roco::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
      std::cout << roco_stringstream.str() << std::endl; \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG_STREAM_FP(level, message) \
    { \
      std::stringstream roco_stringstream; \
      roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
      roco_stringstream << roco::log::getLogColor(level) << "[CTRL " << roco::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << message << roco::log::getResetColor(); \
      std::cout << roco_stringstream.str() << std::endl; \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG_THROTTLE(rate, level, ...) \
    { \
      static double last_hit = 0.0; \
      ::roco::time::TimeStd now = ::roco::time::TimeStd::now(); \
      if (last_hit + rate <= now.toSec()) \
      { \
        last_hit = now.toSec(); \
        ROCO_LOG(level, __VA_ARGS__) \
      } \
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ROCO_LOG_THROTTLE_STREAM(rate, level, message) \
  { \
    static double last_hit = 0.0; \
    ::roco::time::TimeStd now = ::roco::time::TimeStd::now(); \
    if (last_hit + rate <= now.toSec()) \
    { \
      last_hit = now.toSec(); \
      ROCO_LOG_STREAM(level, message) \
    } \
}

} // namespace log
} // namespace roco
