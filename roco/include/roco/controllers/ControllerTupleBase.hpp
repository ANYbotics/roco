/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Gabriel Hottiger
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
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
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
 * @file	ControllerTupleBase.hpp
 * @author	Gabriel Hottiger
 * @date	Aug 15, 2016
 */

#pragma once

// roco
#include "roco/controllers/ControllerSwapStateInterface.hpp"
#include "roco/controllers/ControllerTupleSwapState.hpp"

// STL
#include <algorithm>
#include <exception>
#include <initializer_list>

namespace roco {

template <typename Base_, typename State_, typename Command_, typename... Controllers_>
class ControllerTupleBase : virtual public Base_, public Controllers_... {
 public:
  // This allows returning directly when one of the controllers returns false
  class ControllerTupleException : public std::exception {
    const char* what() const noexcept override { return "Controller tuple exception!"; }
  };

 public:
  //! Default constructor
  ControllerTupleBase() : Base_(), Controllers_()... {}

  //! Default destructor
  ~ControllerTupleBase() override = default;

 protected:
  bool create(double dt) override {
    try {
      std::initializer_list<bool> list = {(Controllers_::create(dt) ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool initialize(double dt) override {
    try {
      std::initializer_list<bool> list = {(Controllers_::initialize(dt) ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool reset(double dt) override {
    try {
      std::initializer_list<bool> list = {(Controllers_::reset(dt) ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool advance(double dt) override {
    try {
      std::initializer_list<bool> list = {(Controllers_::advance(dt) ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool preStop() override {
    try {
      std::initializer_list<bool> list = {(Controllers_::preStop() ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool stop() override {
    try {
      std::initializer_list<bool> list = {(Controllers_::stop() ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool cleanup() override {
    try {
      std::initializer_list<bool> list = {(Controllers_::cleanup() ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool swap(double dt, const ControllerSwapStateInterfacePtr& swapState) override {
    try {
      std::initializer_list<bool> list = {(Controllers_::swap(dt, swapState) ? true : throw(myex_))...};
      (void)list;  // unused warning
    } catch (ControllerTupleException& e) {
      return false;
    }
    return true;
  }

  bool getSwapState(ControllerSwapStateInterfacePtr& swapState) override {
    unsigned int i = 0;
    const std::size_t nrControllers = sizeof...(Controllers_);
    auto* tupleState = new roco::ControllerTupleSwapState(nrControllers);

    try {
      std::initializer_list<bool> list = {(Controllers_::getSwapState(tupleState->getSwapState(i++)) ? true : throw(myex_))...};
      (void)list;  // unused warning
      swapState.reset(std::move(tupleState));
    } catch (ControllerTupleException& e) {
      swapState.reset();
      return false;
    }

    return true;
  }

  bool addSharedModule(const roco::SharedModulePtr& module) override {
    // It is sufficient for one of the controllers to accept the shared module
    std::initializer_list<bool> list = {(Controllers_::addSharedModule(module))...};
    return (std::find(list.begin(), list.end(), true) != list.end());
  }

 private:
  ControllerTupleException myex_;
};

}  // namespace roco
