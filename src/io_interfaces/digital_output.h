#ifndef DIGITAL_OUTPUT_H
#define DIGITAL_OUTPUT_H

#include <bitset>
#include <ros/console.h>
#include "ethercat_interface/write_interface.h"

/**
 * @brief The DigitalOutput class Interface for Digitals Outputs
 */
template<size_t _Nb>
class DigitalOutput: public WriteInterface
{
public:

    /**
     * @brief DigitalOutput Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     * @param data_ptr pointer to the location where the data should be written
     */
    DigitalOutput(std::string name, std::bitset<_Nb> *data_ptr, unsigned int position) :
        data_ptr_(data_ptr), WriteInterface(name), position_(position)
    {data_ptr_->set(position_, false);}

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value)
    {
        uint8* data_int_ptr;
        data_int_ptr = (uint8*)data_ptr_;
        ROS_WARN_STREAM("Data before:" << *data_ptr_ << ", " << *data_int_ptr);
        ROS_WARN("%s: output (before change): %u", name_.c_str(), *data_int_ptr);

        // Write the value
        data_ptr_->set(position_, bool(value));

        // Print debug info
        data_int_ptr = (uint8*)data_ptr_;
        ROS_WARN_STREAM("Raw data at " << data_ptr_ << " after:" << *data_ptr_ << ", " << *data_int_ptr);
        ROS_WARN("%s: output (after change): %u", name_.c_str(), *data_int_ptr);

        return true;
    }

private:

    std::bitset<_Nb>* data_ptr_;
    unsigned int position_;

};  // End of class DigitalOutput

#endif // DIGITAL_OUTPUT_H
