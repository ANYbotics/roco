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
* @file     Controller.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/

#pragma once

// roco
#include "roco/controllers/ControllerAdapteeInterface.hpp"

// boost
#include <boost/thread.hpp>

// STL
#include <string>

namespace roco {
namespace controllers {

//! Controller Implementation
/*! Derive this class and implement your own controller.
 *
 */
template<typename State_, typename Command_>
class Controller: virtual public ControllerAdapteeInterface {
 public:
  //! Typedef of the state of the robot.
  typedef State_ State;
  //! Typedef of the command of the robot.
  typedef Command_ Command;

 public:
  Controller(const std::string& name);
  virtual ~Controller();

  /*! @returns the name of the controller
   */
  virtual const std::string& getName() const;

  /*! Sets the name of the controller.
   * @param name
   */
  virtual void setName(std::string& name);

  //! @returns true if the controller is initialized.
  bool isInitialized() const;

  //! @returns true if the controller has been created.
  bool isCreated() const;

  //! @returns true if the controller is running.
  bool isRunning() const;

  /*! @returns the state of the robot.
   * This method should be implemented by the adapter.
   */
  virtual const State& getState() const = 0;

  /*! @returns a mutex to protect access to the state.
   * This method should be implemented by the adapter.
   */
  virtual boost::shared_mutex& getStateMutex() = 0;

  /*! @returns the command.
   * This method should be implemented by the adapter.
   */
  virtual const Command& getCommand() const = 0;

  /*! @returns a mutex to protect access to the command.
   * This method should be implemented by the adapter.
   */
  virtual boost::shared_mutex& getCommandMutex() = 0;

  /*! @returns the command.
   * This method should be implemented by the adapter.
   */
  virtual Command& getCommand() = 0;

 protected:
  /**
   * @brief
   * @return
   */
  virtual bool stop() { return true; };
  virtual bool prestop() { return true; };


 protected:
  //! Name of the controller
  std::string name_;

  //! Indicates if the controller is created.
  boost::atomic<bool> isCreated_;

  //! Indicates if the controller is initialized.
  boost::atomic<bool> isInitialized_;

  //! Indicates if the controller is running.
  boost::atomic<bool> isRunning_;

};

} /* namespace controllers */
} /* namespace roco */

#include "Controller.tpp"
