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
 * @file     ControllerAdapterExample.hpp
 * @author   Christian Gehring
 * @date     Dec, 2014
 * @brief
 */
#pragma once

#include <exception>  // std::exception
#include <iostream>
#include <memory>

#include <message_logger/message_logger.hpp>

#include <roco/time/Time.hpp>
#include <roco/time/TimeStd.hpp>

#include <roco/controllers/Controller.hpp>
#include <roco/controllers/adapters/ControllerAdapterInterface.hpp>

namespace roco {

template <typename Controller_>
class ControllerAdapterExample : public ControllerAdapterInterface, public Controller_ {
 public:
  using Controller = Controller_;
  using State = typename Controller::State;
  using Command = typename Controller::Command;

 public:
  ControllerAdapterExample() = default;
  ~ControllerAdapterExample() override = default;

  bool createController(double dt) override {
    if (this->isCreated()) {
      return false;
    }
    try {
      this->isCreated_ = this->create(dt);
    } catch (std::exception& e) {
      MELO_WARN_STREAM("Exception caught: " << e.what());
      this->isCreated_ = false;
    }
    return this->isCreated();
  }

  bool initializeController(double dt) override {
    if (!this->isCreated()) {
      return false;
    }
    try {
      this->isInitialized_ = this->initialize(dt);
    } catch (std::exception& e) {
      MELO_WARN_STREAM("Exception caught: " << e.what());
      this->isInitialized_ = false;
    }
    time_.fromSec(0.0);
    *state_ = 0.0;
    return this->isInitialized();
  }

  bool advanceController(double dt) override {
    if (!this->isInitialized()) {
      return false;
    }
    time_ += dt;
    *state_ += 1.0;
    try {
      return this->advance(dt);
    } catch (std::exception& e) {
      MELO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  bool cleanupController() override {
    if (!this->isCreated()) {
      return false;
    }
    try {
      return this->cleanup();
    } catch (std::exception& e) {
      MELO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  bool resetController(double dt) override {
    if (!this->isCreated()) {
      return false;
    }
    if (!this->isInitialized()) {
      return initializeController(dt);
    }

    try {
      return this->reset(dt);
    } catch (std::exception& e) {
      MELO_WARN_STREAM("Exception caught: " << e.what());
    }
    return false;
  }

  bool preStopController() override { return true; }
  bool stopController() override { return true; }

  bool swapController(double dt, const ControllerSwapStateInterfacePtr& /*swapState*/) override { return initializeController(dt); }

  bool getControllerSwapState(ControllerSwapStateInterfacePtr& /*swapState*/) override { return true; }
  bool addControllerSharedModule(const SharedModulePtr& /*module*/) override { return true; }

  bool isControllerInitialized() const override { return true; }
  bool isBeingStopped() const override { return isBeeingStopped_; }
  void setIsBeingStopped(bool isBeeingStopped) override { isBeeingStopped_ = isBeeingStopped; }
  bool isRunning() const override { return isRunning_; }
  void setIsRunning(bool isRunning) override { isRunning_ = isRunning; }

  void setTime(const time::Time& time) override { time_ = time; }

  bool isCheckingCommand() const override { return isCheckingCommand_; }
  void setIsCheckingCommand(bool isChecking) override { isCheckingCommand_ = isChecking; }

  bool isCheckingState() const override { return isCheckingState_; }

  void setIsCheckingState(bool isChecking) override { isCheckingState_ = isChecking; }

  bool isRealRobot() const override { return isRealRobot_; }
  void setIsRealRobot(bool isRealRobot) override { isRealRobot_ = isRealRobot; }

  const std::string& getControllerName() const override { return name_; }

  // State and Command methods
  void setStateAndCommand(std::shared_ptr<State> state, std::shared_ptr<boost::shared_mutex> mutexState, std::shared_ptr<Command> command,
                          std::shared_ptr<boost::shared_mutex> mutexCommand) override {
    stateMutex_ = mutexState;
    commandMutex_ = mutexCommand;
    state_ = state;
    command_ = command;
  }
  const State& getState() const override { return *state_; }
  const Command& getCommand() const override { return *command_; }
  Command& getCommand() override { return *command_; }
  boost::shared_mutex& getStateMutex() override { return *stateMutex_; }
  boost::shared_mutex& getCommandMutex() override { return *commandMutex_; }

  // Worker dummy implementations
  WorkerHandle addWorker(const roco::WorkerOptions& /*options*/) override { return roco::WorkerHandle(); }
  WorkerHandle addWorker(roco::Worker& /*worker*/) override { return roco::WorkerHandle(); }
  bool startWorker(const roco::WorkerHandle& /*workerHandle*/) override { return true; }
  bool stopWorker(const roco::WorkerHandle& /*workerHandle*/, bool /*block*/) override { return true; }
  bool cancelWorker(const roco::WorkerHandle& /*workerHandle*/, bool /*block*/) override { return true; }

  // Time
  const time::Time& getTime() const override { return time_; }

 private:
  //! Indicates if the real robot is controller or only a simulated version.
  std::string name_{"TestController"};
  bool isRealRobot_ = false;
  bool isRunning_ = false;
  bool isBeeingStopped_ = false;
  bool isCheckingCommand_ = true;
  bool isCheckingState_ = true;
  time::TimeStd time_ = time::TimeStd::now();
  std::shared_ptr<State> state_;
  std::shared_ptr<boost::shared_mutex> stateMutex_;
  std::shared_ptr<Command> command_;
  std::shared_ptr<boost::shared_mutex> commandMutex_;
};

} /* namespace roco */
