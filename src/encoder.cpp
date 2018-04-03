#include <ros/console.h>
#include "encoder.h"

//Encoder::Encoder(T *data_ptr) : data_ptr_(data_ptr)
//{
//    previous_value_ = *data_ptr;
//}

//double Encoder::read()
//{
//    uint16_t new_value = *data_ptr_;

//    ROS_DEBUG_THROTTLE(1.0, "Encoder read value: %u", new_value);
//    if( (previous_value_ - new_value) > encoder_max_ / 2)
//    {
//        revolution_overflows_++;
//        ROS_DEBUG_THROTTLE(1.0, "Incrementing overflow to %i", revolution_overflows_);
//    }
//    else if ((previous_value_ - new_value) < -1 * (double)encoder_max_/2)
//    {
//        revolution_overflows_--;
//        ROS_DEBUG_THROTTLE(1.0, "Decrementing overflow to %i", revolution_overflows_);
//    }

//    previous_value_ = new_value;

//    int int_position = revolution_overflows_ * encoder_max_ + new_value;
//    double position = (double) int_position;
//    ROS_DEBUG_THROTTLE(1.0, "Encoder output: %.2f", position);

//    return position;
//}
