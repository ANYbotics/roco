/*!
 * @file	  ControllerSwapStateDummy.hpp
 * @author	Gabriel Hottiger
 * @date	  Mar 10, 2017
 */

#pragma once

// roco
#include "roco/controllers/ControllerSwapStateInterface.hpp"

// STL
#include <iostream>

namespace roco {

class ControllerSwapStateDummy : public ControllerSwapStateInterface
{
 public:
  //! Constructor
  ControllerSwapStateDummy() { }

  //! Destructor
  virtual ~ControllerSwapStateDummy() { }

  /** Overloads the == operator, to compare two states.
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are equal
   */
  virtual bool operator== ( const ControllerSwapStateInterface& state ) const {
    try {
      const ControllerSwapStateDummy &item = dynamic_cast<const ControllerSwapStateDummy&>(state);
      return true;
    }
    catch(const std::bad_cast& e) {
        return false;
    }
  }
};

} /* namespace roco */

