#ifndef DIGITAL_INPUT_H
#define DIGITAL_INPUT_H

#include <ethercat_interface/ethercat_includes.h>  // For uint datatypes
#include <ethercat_interface/read_interface.h>

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
    DigitalInput(std::string name, uint8 *data_ptr, unsigned int position) :
        data_ptr_(data_ptr), ReadInterface(name), position_(position)
    {
    }

    /**
     * @brief read reads the input value and converts it to a double
     * @return 0.0 (false) or 1.0 (true)
     */
    double read();

private:
    uint8* data_ptr_;
    unsigned int position_;

};  // End of class DigitalInput

#endif // DIGITAL_INPUT_H
