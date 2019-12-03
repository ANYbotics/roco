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
 * @file     Time.hpp
 * @author   Christian Gehring
 * @date     Dec, 2014
 * @brief
 */
#pragma once

#include <cstdint>
#include <iomanip>  // std::setw
#include <ostream>

namespace roco {
namespace time {

class Time {
 public:
  //! Default constructor
  Time() = default;
  //! Default destructor
  virtual ~Time() = default;

  /**
   * Create time object from seconds.
   * @param t Time in seconds.
   * @return Time object.
   */
  virtual Time& fromSec(double t) = 0;

  /**
   * Create time object from nanoseconds.
   * @param t Time in nanoseconds.
   * @return Time object.
   */
  virtual Time& fromNSec(uint64_t t) = 0;

  /**
   * Convert time object to seconds.
   * @return Time in seconds
   */
  virtual double toSec() const = 0;

  /**
   * Get seconds component of the time.
   * @return Seconds component of the time.
   */
  virtual uint32_t getSec() const = 0;

  /**
   * Get nanoseconds component of the time.
   * @return Nanoseconds component of the time.
   */
  virtual uint32_t getNSec() const = 0;

  /**
   * Set time to current time.
   * @return Time object.
   */
  virtual Time& setNow() = 0;

  /**
   * Output stream operator.
   * @param out Output stream.
   * @param rhs Time to print.
   * @return Updated output stream.
   */
  friend std::ostream& operator<<(std::ostream& out, const Time& rhs);
};

} /* namespace time */
} /* namespace roco */
