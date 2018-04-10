#ifndef DIGITAL_INPUT_H
#define DIGITAL_INPUT_H

/**
 * @brief The DigitalInput class DigitalInput interface
 */
class DigitalInput : public ReadInterface
{
public:

    /**
     * @brief DigitalInput Constructor.
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    DigitalInput(std::string name, bool *data_ptr) : data_ptr_(data_ptr), ReadInterface(name)
    {
    }

    /**
     * @brief read reads the input value and converts it to a double
     * @return 0.0 (false) or 1.0 (true)
     */
    double read()
    {
        // Read the value
        bool bool_result = *data_ptr_;

        // Convert to double
        double double_result = static_cast<double>(bool_result);

        // Print the result
        ROS_DEBUG_THROTTLE(1.0, "Digital input: %d --> %.1f", bool_result, double_result);

        return double_result;
    }

private:
    bool* data_ptr_;

};  // End of class DigitalInput

#endif // DIGITAL_INPUT_H
