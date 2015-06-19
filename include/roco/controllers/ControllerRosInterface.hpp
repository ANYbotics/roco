#pragma once

namespace roco {
namespace controllers {

class ControllerRosInterface
{
 public:
  ControllerRosInterface() {};
  virtual ~ControllerRosInterface() {};

  virtual void cancelWorkers() {};
  virtual void startWorkers() {};
  virtual bool shutdown() = 0;
};

}
}
