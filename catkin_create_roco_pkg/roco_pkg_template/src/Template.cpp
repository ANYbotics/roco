/*!
* @file 	  ${file_name}.cpp
* @author   ${author}
* @date		  ${date}
* @version 	1.0
* @brief    A controller that ...
*/

// ${pkg_name}
#include "${pkg_name}/${file_name}.h"
${plugin_include}
${plugin_macro}
namespace ${namespace} {

${class_name}::${class_name}()
    : Base()
{
  this->name_ = "${class_name}";
}

${class_name}::~${class_name}() {

}

bool ${class_name}::create(double dt) {
  return true;
}

bool ${class_name}::initialize(double dt) {
  return true;
}

bool ${class_name}::swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState) {
  // ${class_name}:: Call current class reset / initialize
  return isInitialized() ? ${class_name}::reset(dt) : ${class_name}::initialize(dt);
}

bool ${class_name}::getSwapState(roco::ControllerSwapStateInterfacePtr& swapState) {
  swapState.reset(nullptr);
  return true;
}

bool ${class_name}::reset(double dt) {
  return ${class_name}::initialize(dt);
}

bool ${class_name}::advance(double dt) {
  return true;
}

bool ${class_name}::preStop() {
  return true;
}

bool ${class_name}::stop() {
  return true;
}

bool ${class_name}::cleanup() {
  return true;
}

${additional_functions_source}

} /* namespace ${namespace} */
