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
* @file     log_backend.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once


#include "roco/common/assert_macros.hpp"

namespace roco {

namespace log {

namespace levels
{
enum Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
} // namespace levels

typedef levels::Level Level;




const std::string black{"\033[0;30m"};
const std::string red{"\033[0;31m"};
const std::string green     {"\033[0;32m"};
const std::string yellow    {"\033[0;33m"};
const std::string blue      {"\033[0;34m"};
const std::string magenta   {"\033[0;35m"};
const std::string cyan      {"\033[0;36m"};
const std::string white     {"\033[0;37m"};
const std::string def       {"\033[0m"};

const std::string colorFatal = red;
const std::string colorError = red;
const std::string colorWarn = magenta;
const std::string colorInfo = blue;
const std::string colorDebug = cyan;

inline const std::string getResetColor() {
  return def;
}

inline const std::string getLogColor(const roco::log::levels::Level& level) {
  switch (level) {
  case roco::log::levels::Debug:
    return colorDebug;
  case roco::log::levels::Info:
    return colorInfo;
  case roco::log::levels::Error:
    return colorError;
  case roco::log::levels::Warn:
    return colorWarn;
  case roco::log::levels::Fatal:
    return colorFatal;
  default:
    break;
  }
  return def;
}

inline const std::string getLogLevel(const roco::log::levels::Level& level) {
  switch (level) {
  case roco::log::levels::Debug:
    return std::string{"DEBUG"};
  case roco::log::levels::Info:
    return std::string{"INFO"};
  case roco::log::levels::Error:
    return std::string{"ERROR"};
  case roco::log::levels::Warn:
    return std::string{"WARN"};
  case roco::log::levels::Fatal:
    return std::string{"FATAL"};
  default:
    break;
  }
  return std::string{"UNKNOWN"};
}

ROCO_DEFINE_EXCEPTION(roco_fatal, std::runtime_error)
ROCO_DEFINE_EXCEPTION(roco_error, std::runtime_error)


#define ROCO_LOG(level, ...) \
    std::stringstream roco_stringstream; \
    roco_stringstream << roco::log::getLogColor(level) << "[CTRL " << roco::log::getLogLevel(level)  << "] " << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
    std::cout << roco_stringstream.str() << std::endl;

#define ROCO_LOG_STREAM(level, message) \
    std::stringstream roco_stringstream; \
    roco_stringstream << roco::log::colorInfo << "[CTRL " << roco::log::getLogLevel(level)  << "] " << message << roco::log::getResetColor(); \
    std::cout << roco_stringstream.str() << std::endl;

#define ROCO_LOG_FP(level, ...) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << roco::log::colorInfo << "[CTRL " << roco::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << roco::common::internal::roco_string_format(__VA_ARGS__) << roco::log::getResetColor(); \
    std::cout << roco_stringstream.str() << std::endl;

#define ROCO_LOG_STREAM_FP(level, message) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << roco::log::colorInfo << "[CTRL " << roco::log::getLogLevel(level)  << "] " <<  sfp.toString() << " " << message << roco::log::getResetColor(); \
    std::cout << roco_stringstream.str() << std::endl;


#define ROCO_LOG_THROTTLE(rate, level, ...) \
    { \
    static double last_hit = 0.0; \
    ::roco::time::TimeStd now = ::roco::time::TimeStd::now(); \
    if (last_hit + rate <= now.toSec()) \
    { \
      last_hit = now.toSec(); \
      ROCO_LOG(level, __VA_ARGS__) \
    } \
    } \



} // namespace log
} // namespace roco