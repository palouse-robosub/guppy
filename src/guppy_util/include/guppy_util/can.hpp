#ifndef GUPPY_UTIL_CAN_HPP
#define GUPPY_UTIL_CAN_HPP

#include <linux/can.h>

#include <string_view>

namespace guppy_util {

std::string format_can_topic(std::string_view name);

std::string format_can_topic(canid_t id);

} // namespace guppy_util

#endif // GUPPY_UTIL_CAN_HPP
