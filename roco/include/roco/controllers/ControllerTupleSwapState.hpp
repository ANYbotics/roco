/*!
 * @file	  ControllerTupleSwapState.hpp
 * @author	Gabriel Hottiger
 * @date	  Mar 10, 2017
 */

#pragma once

// roco
#include "roco/controllers/ControllerSwapStateInterface.hpp"

// STL
#include <memory>
#include <vector>

namespace roco {

class ControllerTupleSwapState : public ControllerSwapStateInterface {
 public:
  /** Constructor
   *  @param nrStates number of states/controllers
   */
  explicit ControllerTupleSwapState(const std::size_t nrStates) : states_(nrStates) {}

  //! Delete default constructor
  ControllerTupleSwapState() = delete;

  //! Default destructor
  ~ControllerTupleSwapState() override = default;

  /** Overloads the == operator, to compare two states. Returns true by default!
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are equal
   */
  bool operator==(const ControllerSwapStateInterface& state) const override {
    for (auto& stateEntry : states_) {
      if (!(*stateEntry == state)) {
        return false;
      }
    }
    return true;
  }

  /** Overloads the != operator, to compare two states. Returns true by default!
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are not equal
   */
  bool operator!=(const ControllerSwapStateInterface& state) const override {
    for (auto& stateEntry : states_) {
      if (!(*stateEntry != state)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Return swap state of module.
   * @param i Id of the module.
   * @return Swap sate pointer.
   */
  std::unique_ptr<ControllerSwapStateInterface>& getSwapState(const unsigned int i) { return states_.at(i); }

  /**
   * Return swap states of all modules.
   * @return Swapt sates vector.
   */
  const std::vector<std::unique_ptr<ControllerSwapStateInterface> >& getSwapStates() const { return states_; }

  /**
   * Return number of swap states.
   * @return number of swap states.
   */
  std::size_t size() const { return states_.size(); }

 protected:
  //! Swap states for tuple.
  std::vector<std::unique_ptr<ControllerSwapStateInterface> > states_;
};

} /* namespace roco */
