/*
 * LocomotionControllerInterface.hpp
 *
 *  Created on: Jun 19, 2015
 *      Author: dario
 */

#pragma once

namespace roco {
namespace controllers {

class LocomotionControllerInterface :
    virtual public roco::controllers::ControllerInterface
{
 public:

  virtual const std::string getLocomotionModeName() const = 0;

};

}
}
