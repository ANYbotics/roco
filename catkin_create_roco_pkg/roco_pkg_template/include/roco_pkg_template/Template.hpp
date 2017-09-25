/*!
* @file 	  ${file_name}.hpp
* @author   ${author}
* @date		  ${date}
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// ${base_class_package}
#include "${base_class_package}/controllers/controllers.hpp"

// state and command
#include "${state_package}/${state_name}.hpp"
#include "${command_package}/${command_name}.hpp"

namespace ${namespace} {

class ${class_name}: virtual public ${base_class}<${state_package}::${state_name}, ${command_package}::${command_name}>${additional_inheritance} {

 public:
  typedef ${base_class}<${state_package}::${state_name}, ${command_package}::${command_name}> Base;

  //! Construct $class_name.
  ${class_name}();

  //! Destruct $class_name.
  virtual ~${class_name}();

 protected:
  //! Create controller $class_name.
  virtual bool create(double dt);

  //! Initialize controller $class_name.
  virtual bool initialize(double dt);

  //! Advance controller $class_name.
  virtual bool advance(double dt);

  //! Reset controller $class_name.
  virtual bool reset(double dt);

  //! Pre-stop controller $class_name.
  virtual bool preStop();

  //! Stop controller $class_name.
  virtual bool stop();

  //! Cleanup controller $class_name.
  virtual bool cleanup();

  //! Swap to controller $class_name with state 'swap'.
  virtual bool swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState);

  //! Get swap state 'swapState' of controller $class_name.
  virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr& swapState);

  //! Add shared module 'module' to controller $class_name.
  virtual bool addSharedModule(const roco::SharedModulePtr& module);

${additional_functions_header}
};

} /* namespace ${namespace} */
