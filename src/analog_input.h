#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

/**
 * @brief The Analog Input
 */
template <typename T>
class AnalogInput : public ReadInterface
{
public:

    /**
     * @brief AnalogInput Constructor.
     * @param data_ptr pointer to the location where the data should be written
     */
    AnalogInput(std::string name, T *data_ptr) : data_ptr_(data_ptr), ReadInterface(name) {}

    /**
     * @brief read reads the analog input
     * @return the analog input value
     */
    double read()
    {
        T int_value = *data_ptr_;

//        ROS_DEBUG_THROTTLE(1.0, "Encoder read value: %u", new_value);
//        if( (previous_value_ - new_value) > encoder_max_ / 2)
//        {
//            revolution_overflows_++;
//            ROS_DEBUG_THROTTLE(1.0, "Incrementing overflow to %i", revolution_overflows_);
//        }
//        else if ((previous_value_ - new_value) < -1 * (double)encoder_max_/2)
//        {
//            revolution_overflows_--;
//            ROS_DEBUG_THROTTLE(1.0, "Decrementing overflow to %i", revolution_overflows_);
//        }

//        previous_value_ = new_value;

//        int int_position = revolution_overflows_ * encoder_max_ + new_value;
        double value = static_cast<double>(int_value);
        ROS_DEBUG_THROTTLE(1.0, "Analog input: %.1f", value);

        return value;
    }

private:
    T* data_ptr_;

}; // End of class AnalogInput

#endif // ANALOG_INPUT_H
