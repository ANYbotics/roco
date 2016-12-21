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
 * @file     ControllerAdapterInterface.hpp
 * @author   Christian Gehring, Gabriel Hottiger
 * @date     Dec, 2014
 * @note     Restructured, June 2016
 */

#pragma once

// STL
#include <string>

namespace roco {

/*! Abstract interface class for controller adapters.
 *
 *  Derive this class and implement your own controller adapter.
 */
class ControllerAdapterInterface
{
 public:

  //! Empty constructor
  ControllerAdapterInterface() { }

  //! Empty constructor
  virtual ~ControllerAdapterInterface() { }

  /*! Adapts the adaptees create(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool createController(double dt) = 0;

  /*! Adapts the adaptees initialize(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool initializeController(double dt) = 0;

  /*! Adapts the adaptees advance(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool advanceController(double dt) = 0;

  /*! Adapts the adaptees reset(dt) function.
   * @param dt  time step [s]
   * @returns true if successful
   */
  virtual bool resetController(double dt) = 0;

  /*! Adapts the adaptees prestop(dt) function.
   * @returns true if successful
   */
  virtual bool preStopController() = 0;

  /*! Adapts the adaptees stop(dt) function.
   * @returns true if successful
   */
  virtual bool stopController() = 0;

  /*! Adapts the adaptees cleanup() function.
   * @returns true if successful
   */
  virtual bool cleanupController() = 0;

  /*! Sets if the real robot is controlled or only a simulated version.
   * @param flag indicating robot type
   */
  virtual void setIsRealRobot(bool isRealRobot) = 0;

  /*! This function gets the name of the controller.
   * @returns controller name
   */
  virtual const std::string& getControllerName() const = 0;

  /*! This function indicates whether the controller was initialized.
   * @returns true iff controller is initialized
   */
  virtual bool isControllerInitialized() const = 0;

  /*! This function indicates whether the controller is being stopped
   * @returns true iff controller is being stopped
   */
  virtual bool isBeingStopped() const = 0;

  /*! This function indicates whether the controller is being stopped
   * @returns true iff controller is being stopped
   */
  virtual void setIsBeingStopped(bool isBeeingStopped) = 0;




};

} /* namespace roco */