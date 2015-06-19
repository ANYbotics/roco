/*
 * WorkerOptions.hpp
 *
 *  Created on: Jun 18, 2015
 *      Author: dario
 */

#pragma once

#include <roco/workers/WorkerEvent.hpp>
#include <string>
#include <boost/function.hpp>

namespace roco {

typedef boost::function<bool(const WorkerEvent&)> WorkerCallback;

class WorkerOptions
{

 public:
  WorkerOptions()
      : frequency_(1.0),
        autostart_(false),
        synchronous_(false),
        callback_(0),
        name_(""),
        priority_(0)
  { }

  virtual ~WorkerOptions() { }

  double frequency_;
  bool autostart_;
  bool synchronous_;
  WorkerCallback callback_;
  int priority_;
  std::string name_;

};

}
