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

public:
  /**
   * @brief Default Constructor $class_name
   */
  ${class_name}();

  /**
   * @brief Default Destructor $class_name
   */
  virtual ~${class_name}();

protected:

  /*! Use this method instead of the constructor to create objects.
  * This method is only called once during the whole lifetime of the controller.
  * @param dt  time step [s]
  * @returns true if successful
  */
 virtual bool create(double dt);

 /*! This initializes the controller before the advance method is called.
  * @param dt  time step [s]
  * @returns true if successful
  */
 virtual bool initialize(double dt);

 /*! This advances the controller.
  * @param dt  time step [s]
  * @returns true if successful
  */
 virtual bool advance(double dt);

 /*! This resets the controller assuming it was already initialized once.
  * @param dt  time step [s]
  * @returns true if successful
  */
 virtual bool reset(double dt);

 /*! This prepares the controller for a stop/switch. Unregister, delete everything that
  *  is not essential to advance (e.g. ros communication)
  * @returns true if successful
  */
 virtual bool preStop();

 /*! This stops the controller. Kill everything that is not used when controller is not
  *  advancing anymore. (E.g running threads etc.)
  * @returns true if successful
  */
 virtual bool stop();

 /*! Use this method instead of the destructor to destroy objects.
  * This method is only called once at the end of the lifetime of the controller.
  * @returns true if successful
  */
 virtual bool cleanup();

 /*! Use this method to swap from another controller.
  * @param dt  time step [s]
  * @param state  State received from the previous controller
  * @returns true if successful
  */
 virtual bool swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState);

 /*! Use this method to get the state of the controller. Must be thread-safe parallel to advance.
  * @param   swapState reference to state to be set
  * @returns true if successful
  */
 virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr& swapState);

 /*! Use this method to set a shared module to the controller.
  * @param   module reference to module to be set
  */
 virtual void addSharedModule(const roco::SharedModulePtr& module);

${additional_functions_header}
};

} /* namespace ${namespace} */
