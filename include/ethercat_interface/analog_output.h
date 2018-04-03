#ifndef ANALOG_OUTPUT_H
#define ANALOG_OUTPUT_H

#include "ethercat_interface/write_interface.h"

/**
 * @brief The AO class Interface for Analog Outputs
 */
class AO: public WriteInterface
{
public:
    /**
     * @brief AO Constructor
     * @param data_ptr pointer to the location where the data should be written
     * @param nr_bits number of bits used for conversion
     * @param min minimum output value (before conversion from double)
     * @param max maximum output value (before conversion from double)
     */
    AO(int16_t *data_ptr, unsigned int nr_bits, double min, double max);

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value);

private:

    // ToDo 1: is int16_t the right data type
    // ToDo 2: can we move this data_ptr_ to the base class?
    int16_t* data_ptr_;
    unsigned int nr_bits_;
    double min_, max_;
}; // End of class AO

#endif // ANALOG_OUTPUT_H
