#include <ros/console.h>
#include <bitset>

#include "digital_input.h"

double DigitalInput::read()
{
    // Read the value from the memory
    std::bitset<8> bitset = *data_ptr_;

    // Read the value
    bool bool_result = bitset[position_];

    // Convert to double
    double double_result = static_cast<double>(bool_result);

    // Print the result
    ROS_DEBUG_THROTTLE(1.0, "Digital input: %d --> %.1f", bool_result, double_result);

    return double_result;
}
