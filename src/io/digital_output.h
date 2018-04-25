#pragma once

#include <bitset>
#include <ros/console.h>
#include "ethercat_interface/output.h"

namespace ethercat_interface
{
/**
 * @brief The DigitalOutput class Interface for Digitals Outputs
 */
template <size_t _Nb>
class DigitalOutput : public Output
{
public:
  /**
   * @brief DigitalOutput Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   */
  DigitalOutput(std::string name, std::bitset<_Nb>* data_ptr, size_t position)
    : data_ptr_(data_ptr), Output(name), position_(position)
  {
    data_ptr_->set(position_, false);
  }

  /**
   * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
   * @param value value to write
   * @return bool indicating success
   */
  bool write(double value)
  {
    // Write the value
    data_ptr_->set(position_, bool(value));
    return true;
  }

private:
  std::bitset<_Nb>* data_ptr_;
  size_t position_;

};  // End of class DigitalOutput

}  // namespace ethercat_interface
