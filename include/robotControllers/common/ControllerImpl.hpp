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
* @file     ControllerImpl.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/

#pragma once

#include "robotControllers/common/ControllerInterface.hpp"
#include <string>

namespace robotControllers {

//! Controller Implementation
/*! Derive this class and implement your own controller.
 *
 */
class ControllerImpl: public virtual ControllerInterface {
 public:
  ControllerImpl(const std::string& name = std::string{""}, bool isRealRobot = false);
  virtual ~ControllerImpl();

  const std::string& getName() const;
  bool isInitialized() const;
  bool isCreated() const;
  bool isRealRobot() const;

 protected:
  /*! Use this method instead of the constructor to create objects.
   * This method is only called once during the whole lifetime of the controller.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool create(double dt) = 0;

  /*! This method is invoked
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;
  virtual bool cleanup() = 0;
  virtual bool reset() = 0;

 protected:
  //! Name of the controller
  std::string name_;
  //! Indicates if the controller is created.
  bool isCreated_;
  //! Indicates if the controller is initialized.
  bool isInitialized_;

  bool isRealRobot_;
};


} /* namespace robotControllers */

