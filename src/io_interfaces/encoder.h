#ifndef ENCODER_H
#define ENCODER_H

#include <limits>
#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/read_interface.h"

/**
 * @brief The Encoder class Encoder interface
 */
template <typename T>
class Encoder : public ReadInterface
{
public:

    /**
     * @brief Encoder Constructor. Handles overflow, assuming raw data is a 16-bit unsigned integer
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    Encoder(std::string name, T *data_ptr) : data_ptr_(data_ptr), ReadInterface(name)
    {
        previous_value_ = *data_ptr;
        encoder_max_ = std::numeric_limits<T>::max();
        ROS_DEBUG_STREAM("Encoder max: " << encoder_max_);
    }

    /**
     * @brief read reads the encoder value, keeps track of overflow and converts to an int
     * @return the current encoder position (in counts)
     */
    double read()
    {
        T new_value = *data_ptr_;

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

        int int_position = revolution_overflows_ * encoder_max_ + new_value;
        double position = static_cast<double>(int_position);
        ROS_DEBUG_THROTTLE(1.0, "Encoder output: %.1f", position);

        return position;
    }

private:
    T* data_ptr_;

    T previous_value_;  //Value of this Encoder at the previous read action
    int revolution_overflows_ = 0; // How many times has the encoder encountered a full revolution
    T encoder_max_ = 0; // The largest value this encoder can report before wrapping around and to a full revolution

};  // End of class Encoder

#endif // ENCODER_H
