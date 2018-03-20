#include <iostream>
#include <ros/console.h>

#include "ethercat_interface/io_interfaces.h"
#include "ethercat_interface/ethercat_includes.h"


AO::AO(int16_t *data_ptr, unsigned int nr_bits, double min, double max):
    data_ptr_(data_ptr), nr_bits_(nr_bits), min_(min), max_(max)
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

int AO::read()
{

}

Encoder::Encoder(uint16_t *data_ptr) : data_ptr_(data_ptr)
{
    previous_value_ = *data_ptr;
}

int Encoder::read()
{
    uint16_t new_value = *data_ptr_;

    ROS_DEBUG_THROTTLE(1.0, "Encoder read value: %u", new_value);
    if( (previous_value_ - new_value) > encoder_max_ / 2)
    {
        revolution_overflows_++;
        ROS_DEBUG_THROTTLE(1.0, "Incrementing overflow to %i", revolution_overflows_);
    }
    else if ((previous_value_ - new_value) < -1 * (double)encoder_max_/2)
    {
        revolution_overflows_--;
        ROS_DEBUG_THROTTLE(1.0, "Decrementing overflow to %i", revolution_overflows_);
    }

    previous_value_ = new_value;

    int position = revolution_overflows_ * encoder_max_ + new_value;
    ROS_DEBUG_THROTTLE(1.0, "Encoder output: %i", position);

    return position;
}
