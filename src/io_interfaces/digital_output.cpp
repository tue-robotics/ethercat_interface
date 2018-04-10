#include <ros/console.h>
#include <bitset>

#include "digital_output.h"

bool DigitalOutput::write(double value)
{
    // Convert the data pointer to a bitset pointer
    std::bitset<8>* bitset = (std::bitset<8>*)data_ptr_;

    // Write the value
    bitset[position_] = (bool)value;

    return true;
}
