#include "guppy_util/can.hpp"

#include <sstream>
#include <ios>

namespace guppy_util {

[[maybe_unused]] std::string format_can_topic(std::string_view name) {
    static constexpr const auto base_name = "/can/";
    std::stringstream           stream;
    stream << base_name << name;
    return stream.str();
}

[[maybe_unused]] std::string format_can_topic(canid_t id) {
    static constexpr const auto base_id = "/can/unkown/id_0x";
    std::stringstream           stream;
    stream << base_id << std::hex << id;
    return stream.str();
}

}    // namespace guppy_util
