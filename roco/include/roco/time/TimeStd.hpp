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
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/
/*!
 * @file     TimeStd.hpp
 * @author   Christian Gehring
 * @date     Dec, 2014
 * @brief
 */
#pragma once

#include <cstdint>
#include <roco/time/Time.hpp>

namespace roco {
namespace time {

class TimeStd : virtual public Time {
 public:
  //! Default constructor
  TimeStd() = default;
  /**
   * Construct time from seconds and nanoseconds parts.
   * @param sec Seconds.
   * @param nsec Nanoseconds.
   */
  TimeStd(uint32_t sec, uint32_t nsec);

  /**
   * Construct time from nanoseconds.
   * @param nanoseconds Time in nanoseconds.
   */
  explicit TimeStd(uint64_t nanoseconds);

  /**
   * Construct time from seconds.
   * @param nanoseconds Time in seconds.
   */
  explicit TimeStd(double seconds);

  //! Copy constructor
  explicit TimeStd(const Time& time);

  //! Default destructor
  ~TimeStd() override = default;

  // Operator overloads
  TimeStd& operator=(const Time& time);
  TimeStd& operator=(const TimeStd& rhs);
  TimeStd operator+(const TimeStd& rhs) const;
  TimeStd operator-(const TimeStd& rhs) const;
  TimeStd operator-() const;
  TimeStd& operator+=(const TimeStd& rhs);
  TimeStd& operator-=(const TimeStd& rhs);
  TimeStd operator+(double t) const;
  TimeStd& operator+=(double t);

  /**
   * Set time object from seconds and nanoseconds.
   * @param sec Time seconds part.
   * @param nsec Time nanoseconds part.
   * @return Time object.
   */
  TimeStd& from(uint32_t sec, uint32_t nsec);

  //! @copydoc Time::fromSec
  Time& fromSec(double t) override;
  //! @copydoc Time::fromNSec
  Time& fromNSec(uint64_t t) override;
  //! @copydoc Time::toSec
  double toSec() const override;
  //! @copydoc Time::getSec
  uint32_t getSec() const override;
  //! @copydoc Time::getNSec
  uint32_t getNSec() const override;
  //! @copydoc Time::setNow
  Time& setNow() override;

  //! Get current time.
  static TimeStd now();

  /**
   * Output stream operator.
   * @param out Output stream.
   * @param rhs Time to print.
   * @return Updated output stream.
   */
  friend std::ostream& operator<<(std::ostream& out, const TimeStd& rhs);

 protected:
  inline void normalizeSecNSec(uint64_t& sec, uint64_t& nsec) const;
  inline void normalizeSecNSec(uint32_t& sec, uint32_t& nsec) const;
  inline void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec) const;

 protected:
  uint32_t sec_ = 0u;
  uint32_t nsec_ = 0u;
};

} /* namespace time */
}  // namespace roco
