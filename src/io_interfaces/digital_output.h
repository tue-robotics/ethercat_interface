#ifndef DIGITAL_OUTPUT_H
#define DIGITAL_OUTPUT_H

#include <bitset>
#include "ethercat_interface/write_interface.h"

/**
 * @brief The AnalogOutput class Interface for Analog Outputs
 */
template<size_t _Nb>
class DigitalOutput: public WriteInterface
{
public:

    /**
     * @brief AnalogOutput Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    DigitalOutput(std::string name, std::bitset<_Nb> *data_ptr, unsigned int position) :
        data_ptr_(data_ptr), WriteInterface(name), position_(position)
    {}

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value)
    {
        // Write the value
        (*data_ptr_)[position_] = (bool)value;

        return true;
    }

private:

    std::bitset<_Nb>* data_ptr_;
    unsigned int position_;

};  // End of class DigitalOutput

#endif // DIGITAL_OUTPUT_H
