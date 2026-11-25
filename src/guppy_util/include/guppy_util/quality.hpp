#ifndef GUPPY_UTIL_QUALITY_HPP
#define GUPPY_UTIL_QUALITY_HPP

#include "rclcpp/qos.hpp"

namespace guppy_util {

namespace quality {

static inline const auto keep_last_profile =
    rclcpp::QoS(10).reliable().transient_local().keep_last(1);
static inline const auto reliable_profile = rclcpp::QoS(10).reliable();
static inline const auto volatile_profile = rclcpp::QoS(10).best_effort().durability_volatile();

} // namespace quality

} // namespace guppy_util

#endif // GUPPY_UTIL_QUALITY_HPP
