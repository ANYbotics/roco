#pragma once

#include "roco/common/assert_macros.hpp"


namespace roco {
namespace common {

ROCO_DEFINE_EXCEPTION(roco_fatal, std::runtime_error)
ROCO_DEFINE_EXCEPTION(roco_error, std::runtime_error)


#define ROCO_FATAL(...) \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << roco::common::internal::roco_string_format(__VA_ARGS__); \
    roco::common::internal::roco_throw_exception<roco::common::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str());

#define ROCO_FATAL_STREAM(message) \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << message; \
    roco::common::internal::roco_throw_exception<roco::common::roco_fatal>("[CTRL FATAL] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str());

#define ROCO_ERROR(...) \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << roco::common::internal::roco_string_format(__VA_ARGS__); \
    roco::common::internal::roco_throw_exception<roco::common::roco_fatal>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str());

#define ROCO_ERROR_STREAM(message) \
    std::stringstream roco_assert_stringstream;             \
    roco_assert_stringstream << message; \
    roco::common::internal::roco_throw_exception<roco::common::roco_error>("[CTRL ERROR] ", __FUNCTION__,__FILE__,__LINE__, roco_assert_stringstream.str());

#define ROCO_INFO(...) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << "[CTRL INFO] " <<  sfp.toString() << " " << roco::common::internal::roco_string_format(__VA_ARGS__); \
    std::cout << roco_stringstream.str() << std::endl;

#define ROCO_INFO_STREAM(message) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << "[CTRL INFO] " <<  sfp.toString() << " " << message; \
    std::cout << roco_stringstream.str() << std::endl;

#ifndef NDEBUG
#define ROCO_DEBUG(...) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << "[CTRL DEBUG] " <<  sfp.toString() << " " << roco::common::internal::roco_string_format(__VA_ARGS__); \
    std::cout << roco_stringstream.str() << std::endl;

#define ROCO_DEBUG_STREAM(message) \
    std::stringstream roco_stringstream; \
    roco::common::internal::source_file_pos sfp(__FUNCTION__,__FILE__,__LINE__); \
    roco_stringstream << "[CTRL DEBUG] " <<  sfp.toString() << " " << message; \
    std::cout << roco_stringstream.str() << std::endl;
#else
#define ROCO_DEBUG(...)
#define ROCO_DEBUG_STREAM(message)
#endif

} // namespace common
} // namespace roco
