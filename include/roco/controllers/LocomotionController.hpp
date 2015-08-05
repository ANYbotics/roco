/*
 * LocomotionController.hpp
 *
 *  Created on: Jun 19, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <roco/controllers/LocomotionControllerInterface.hpp>

namespace roco {
namespace controllers {

class LocomotionController : public roco::controllers::LocomotionControllerInterface {
 public :
  LocomotionController() {}
  virtual ~LocomotionController() {}
  virtual const std::string getLocomotionModeName() const { return "none"; }
};

}
}
