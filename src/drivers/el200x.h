#pragma once

#include <bitset>
#include "ethercat_interface/driver.h"
#include "../io/digital_output.h"

namespace ethercat_interface
{
/**
 * @brief The EL200x class base class for drivers for Beckhoff Digital outputs
 * The 'x' represents the number of outputs: 2, 4 or 8
 */
template <size_t _Nc>
class EL200x : public Driver
{
public:
  /**
   * @brief EL200x Constructor
   * @param name name of this slave
   * @param slave memory location of the internal struct
   * @param nr_channels nr of analog output channels
   */
  EL200x(ec_slavet* slave) : Driver(slave)
  {
    std::bitset<8>* data_ptr = (std::bitset<8>*)&(((out_el2xxxt*)(slave->outputs))->bits);  // ToDo: check

    // Set everything to 1  // ToDo: is this necessary?
    *data_ptr = 255;

    for (size_t output_nr = 0; output_nr < _Nc; output_nr++)
    {
      ROS_DEBUG("EL200x: Getting data ptr %zu", output_nr);

      std::string channel_name = "Digital output " + std::to_string(output_nr);
      outputs_[output_nr] = std::make_shared<DigitalOutput<8> >(channel_name, data_ptr, output_nr + slave->Ostartbit);
    }
  }

private:
  typedef struct PACKED
  {
    uint8 bits;
  } out_el2xxxt;

};  // End of class EL200x

typedef EL200x<2> EL2002;
typedef EL200x<4> EL2004;
typedef EL200x<8> EL2008;

}  // namespace ethercat_interface
