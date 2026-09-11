#include <gpiod.hpp>
#include "dataCache.hpp"

#define CHIP_PATH "/dev/gpiochip0"
#define LINE_OFFSET 23

class P10DLB
{
public:
    P10DLB();
    ~P10DLB() {}
    bool readValue();
    std::string JSONOutput();

private:
    gpiod::chip chip;
    gpiod::line_request request;
};