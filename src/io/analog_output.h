#pragma once

#include <ros/console.h>
#include "ethercat_interface/output.h"

namespace ethercat_interface
{
template <typename T>
/**
 * @brief The AnalogOutput class Interface for Analog Outputs
 */
class AnalogOutput : public Output
{
public:
  /**
   * @brief AnalogOutput Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   * @param nr_bits number of bits used for conversion
   * @param min minimum output value (before conversion from double)
   * @param max maximum output value (before conversion from double)
   * @param offset integer offset, used if 0 V does not correspond to 0 bits
   */
  AnalogOutput(std::string name, T* data_ptr, size_t nr_bits, double min, double max, T offset = 0)
    : data_ptr_(data_ptr), nr_bits_(nr_bits), min_(min), max_(max), offset_(offset), Output(name)
  {
  }

  /**
   * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
   * @param value value to write
   * @return bool indicating success
   */
  bool write(double value)
  {
    // Convert voltage setpoint to dac value,
    // Last one and first three bits not used
    // ToDo: double check!!!
    T dac_setpoint = value * (1 << nr_bits_) / (max_ - min_) + offset_;
    ROS_DEBUG_THROTTLE(1.0, "Writing value %.2f to %i", value, dac_setpoint);
    *data_ptr_ = (T)dac_setpoint;
    return true;
  }

private:
  T* data_ptr_;
  size_t nr_bits_;
  T offset_;
  double min_, max_;

};  // End of class AO

}  // namespace ethercat_interface
