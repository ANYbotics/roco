/*
 * WorkerEvent.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: dario
 */

#pragma once

#include <roco/time/TimeStd.hpp>

namespace roco {
  /** \brief ROS worker event
    *
    * This class is passed as a parameter to the worker event.
    */
  class WorkerEvent {
  public:
    /** \brief Default constructor
      */
    WorkerEvent() {}

    /** \brief The expected cycle time of the worker
      */
    time::TimeStd expectedCycleTime_;

    /** \brief The momentary, actual cycle time of the worker
      */
    time::TimeStd actualCycleTime_;
  };
};
