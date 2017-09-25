/*!
* @file 	  ${pkg_name}_plugin.cpp
* @author   ${author}
* @date		  ${date}
* @version 	1.0
* @brief    Plugin export for controller ${class_name}.
*/

// state and command
#include "${state_package}/${state_name}.hpp"
#include "${command_package}/${command_name}.hpp"

// ${pkg_name}
#include "${pkg_name}/${file_name}.h"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
${plugin_macro}
