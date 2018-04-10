#ifndef DIGITAL_OUTPUT_H
#define DIGITAL_OUTPUT_H

#include <ros/console.h>
#include "ethercat_interface/write_interface.h"

/**
 * @brief The AnalogOutput class Interface for Analog Outputs
 */
class DigitalOutput: public WriteInterface
{
public:

    /**
     * @brief AnalogOutput Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    DigitalOutput(std::string name, bool *data_ptr) :
        data_ptr_(data_ptr), WriteInterface(name)
    {}

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value)
    {
        // Convert double value to bool value
        // ToDo: do we really also want to use doubles here as well???
        bool bool_value = (bool)value;
        ROS_DEBUG_THROTTLE(1.0, "Writing value %.2f to %d", value, bool_value);
        *data_ptr_ = bool_value;
        return true;
    }

private:

    bool* data_ptr_;

};  // End of class DigitalOutput

#endif // DIGITAL_OUTPUT_H
