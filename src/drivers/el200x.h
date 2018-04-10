#ifndef EL200X_H
#define EL200X_H

#include <bitset>
#include "ethercat_interface/ethercat_driver.h"
#include "../io_interfaces/digital_output.h"

/**
 * @brief The EL200x class base class for drivers for Beckhoff Digital outputs
 * The 'x' represents the number of outputs: 2, 4 or 8
 */
template<size_t _Nc>
class EL200x: public EtherCatDriver
{
public:

    /**
     * @brief EL200x Constructor
     * @param name name of this slave
     * @param slave memory location of the internal struct
     * @param nr_channels nr of analog output channels
     */
    EL200x(std::string name, ec_slavet *slave)// , unsigned int nr_channels);
    {
        std::bitset<8>* data_ptr = slave->outputs;  // ToDo: check
        for (unsigned int output_nr = 0; output_nr < _Nc; output_nr++)
        {
            ROS_DEBUG("EL200x: Getting data ptr %u", output_nr);

            std::string channel_name = "Digital output " + std::to_string(output_nr);
            outputs_[output_nr] = std::make_shared<DigitalOutput<_Nc> >(channel_name, data_ptr, output_nr);
        }
    }

};  // End of class EL200x

typedef EL200x<2> EL2002;
typedef EL200x<4> EL2004;
typedef EL200x<8> EL2008;

#endif // EL200x
