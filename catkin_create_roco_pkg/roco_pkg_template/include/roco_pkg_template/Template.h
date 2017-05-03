/*!
* @file 	  ${class_name}.hpp
* @author   ${author}
* @date		  ${date}
* @version 	1.0
* @brief    A controller that ...
*/

#pragma once

// roco
#include "roco/controllers/controllers.hpp"

// state and command
 #include "${state_package}/${state_name}.hpp"
 #include "${command_package}/${command_name}.hpp"

namespace ${namespace} {

class ${class_name}: virtual public roco::Controller<${state_package}::${state_name}, ${command_package}::${command_name}> {
public:
    typedef roco::Controller<${state_package}::${state_name}, ${command_package}::${command_name}> Base;

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
    /**
    * @brief Create controller $class_name
    * @param dt  Controller time step
    */
    virtual bool create(double dt);


    /**
     * @brief Initialize controller $class_name
     * @param dt  Controller time step
     */
    virtual bool initialize(double dt);

    /**
     * @brief Swap from other controller to controller $class_name
     * @param swapState  Unique ptr containing the state
     */
    virtual bool swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState);

    /**
     * @brief Get state of controller $class_name
     * @param swapState  Unique ptr for storing the state
     */
    virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr& swapState);

    /**
     * @brief Reset controller $class_name
     * @param dt  Controller time step
     */
    virtual bool reset(double dt);

    /**
     * @brief Advance controller $class_name
     * @param dt  Controller time step
     */
    virtual bool advance(double dt);

    /**
     * @brief Pre-Stop controller $class_name
     */
    virtual bool preStop();

    /**
     * @brief Stop controller $class_name
     */
    virtual bool stop();

    /**
     * @brief Clean up $class_name
     */
    virtual bool cleanup();
};

} /* namespace ${namespace} */
