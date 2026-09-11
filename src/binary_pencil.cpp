#include <gpiod.hpp>
#include "binary_pencil.hpp"

P10DLB::P10DLB()
    : chip("/dev/gpiochip0"),
      request(chip.prepare_request()
          .set_consumer("p10dlb-limit-switch")
          .add_line_settings(23, gpiod::line_settings().set_direction(gpiod::line::direction::INPUT))
          .do_request()) {}

bool P10DLB::readValue()
{
    gpiod::line::value value = request.get_value(LINE_OFFSET);
    bool active = (value == gpiod::line::value::ACTIVE);
    return active;
}

std::string P10DLB::JSONOutput()
{
    bool active = readValue();
    std::ostringstream oss;
    oss << "{ \"active\": " << active << " }";
    return oss.str();
}