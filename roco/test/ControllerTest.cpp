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
 * @file     example_main.cpp
 * @author   Christian Gehring
 * @date     Dec, 2014
 * @brief
 */

#include "include/ControllerAdapterExample.hpp"
#include "include/ControllerExample.hpp"

#include <iostream>

#include <gtest/gtest.h>

TEST(RocoController, testInterface) {  // NOLINT
  double dt = 0.1;

  auto state = std::make_shared<double>(0.0);
  auto stateMutex = std::make_shared<boost::shared_mutex>();
  auto command = std::make_shared<double>(0.0);
  auto commandMutex = std::make_shared<boost::shared_mutex>();

  roco::ControllerAdapterExample<roco::ControllerExample> controller;
  controller.setStateAndCommand(state, stateMutex, command, commandMutex);

  std::cout << controller.getName() << std::endl;
  ASSERT_TRUE(controller.createController(dt));

  ASSERT_TRUE(controller.initializeController(dt));

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(controller.advanceController(dt));
  }

  ASSERT_TRUE(controller.cleanupController());
}
