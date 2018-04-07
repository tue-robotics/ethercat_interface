#include <iostream>
#include <ros/console.h>

#include "ethercat_interface/analog_output.h"
#include "ethercat_interface/ethercat_includes.h"

std::ostream &operator<<(std::ostream& strm, const IOInterface& obj)
{
    strm << obj.name_;
    return strm;
}

AO::AO(std::string name, int16_t *data_ptr, unsigned int nr_bits, double min, double max):
    data_ptr_(data_ptr), nr_bits_(nr_bits), min_(min), max_(max), WriteInterface(name)
{

}

bool AO::write(double value)
{
    // Convert voltage setpoint to dac value,
    // Last one and first three bits not used
    // ToDo: double check!!!
    int16_t dac_setpoint = value * (1 << nr_bits_) / (max_ - min_);
    ROS_DEBUG_THROTTLE(1.0, "Writing value %.2f to %i", value, dac_setpoint);
    *data_ptr_ = (int16_t) dac_setpoint;
}

