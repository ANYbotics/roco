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

// ROCO
#include "roco/controllers/ControllerInterface.hpp"

namespace roco {

/*! Abstract interface class for controller adapters.
 *
 *  Derive this class and implement your own controller adapter.
 */
class ControllerAdapterInterface: public ControllerInterface
{
 public:

  //! Empty constructor
  ControllerAdapterInterface() {};
  //! Empty constructor
  virtual ~ControllerAdapterInterface() {};


  /**
   *  These functions are adapting/extending the functions given in the ControllerAdapteeInterface.
   *  E.g. "bool createController(double dt)" adapts "bool create(double dt)".
   */

  //! Create controller procedure
  virtual bool createController(double dt) = 0;
  //! Initialize controller procedure
  virtual bool initializeController(double dt) = 0;
  //! Advance controller procedure
  virtual bool advanceController(double dt) = 0;
  //! Cleanup controller procedure
  virtual bool cleanupController() = 0;
  //! Reset controller procedure
  virtual bool resetController(double dt) = 0;
  //! Stop controller procedure
  virtual bool stopController() = 0;
  //! Prepare stop controller procedure
  virtual bool preStopController() = 0;
};

} /* namespace roco */
