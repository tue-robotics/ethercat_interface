#ifndef DIGITAL_OUTPUT_H
#define DIGITAL_OUTPUT_H

#include <ros/console.h>
#include "ethercat_interface/ethercat_includes.h"
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
    DigitalOutput(std::string name, uint8 *data_ptr, unsigned int position) :
        data_ptr_(data_ptr), WriteInterface(name), position_(position)
    {}

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value);

private:

    uint8* data_ptr_;  // ToDo: replace by bitset?
    unsigned int position_;

};  // End of class DigitalOutput

#endif // DIGITAL_OUTPUT_H
