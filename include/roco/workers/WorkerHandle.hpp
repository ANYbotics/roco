/*
 * WorkerHandle.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: dario
 */

#pragma once

#include <string>

namespace roco {

class WorkerHandle {
  public:
    WorkerHandle() {}
    virtual ~WorkerHandle() {}

    std::string name_;
};

}
