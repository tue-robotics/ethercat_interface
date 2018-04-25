#pragma once

#include "ethercat_interface/input.h"

namespace ethercat_interface
{
/**
 * @brief The Analog Input
 */
template <typename T>
class AnalogInput : public Input
{
public:
  /**
   * @brief AnalogInput Constructor.
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   */
  AnalogInput(std::string name, T* data_ptr) : data_ptr_(data_ptr), Input(name)
  {
  }

  /**
   * @brief read reads the analog input
   * @return the analog input value
   */
  double read()
  {
    T int_value = *data_ptr_;

    double value = static_cast<double>(int_value);
    ROS_DEBUG_THROTTLE(1.0, "Analog input: %.1f", value);

    return value;
  }

private:
  T* data_ptr_;

};  // End of class AnalogInput

}  // namespace ethercat_interface
