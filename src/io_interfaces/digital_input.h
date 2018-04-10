#ifndef DIGITAL_INPUT_H
#define DIGITAL_INPUT_H

#include <bitset>
#include <ethercat_interface/read_interface.h>

/**
 * @brief The DigitalInput class DigitalInput interface
 */
template<size_t _Nb>
class DigitalInput : public ReadInterface
{
public:

    /**
     * @brief DigitalInput Constructor.
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    DigitalInput(std::string name, std::bitset<_Nb> *data_ptr, unsigned int position) :
        data_ptr_(data_ptr), ReadInterface(name), position_(position)
    {
    }

    /**
     * @brief read reads the input value and converts it to a double
     * @return 0.0 (false) or 1.0 (true)
     */
    double read()
    {
        // Read the value
        bool bool_result = (*data_ptr_)[position_];

        // Convert to double
        double double_result = static_cast<double>(bool_result);

        // Print the result
        ROS_DEBUG_THROTTLE(1.0, "Digital input %s: %d --> %.1f", name_.c_str(), bool_result, double_result);

        return double_result;
    }

private:
    std::bitset<_Nb>* data_ptr_;
    unsigned int position_;

};  // End of class DigitalInput

#endif // DIGITAL_INPUT_H
