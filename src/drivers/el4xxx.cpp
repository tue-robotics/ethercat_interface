#include "./el4xxx.h"
#include "../io/analog_output.h"

namespace ethercat_interface
{
EL4xxx::EL4xxx(ec_slavet* slave, size_t nr_channels, size_t nr_bits, double min, double max) : Driver(slave)
{
  ROS_INFO("Constructing EL4xxx with %zu channels, %zu bits, range [%.2f, %.2f]", nr_channels, nr_bits, min, max);

  ROS_DEBUG("EL4xxx: Constructing output objects");
  for (size_t output_nr = 0; output_nr < nr_channels; output_nr++)
  {
    ROS_DEBUG("EL4xxx: Getting data ptr %zu", output_nr);
    // 16 bit int vs 8 bit char
    int16_t* data_ptr = (int16_t*)&(slave->outputs[2 * output_nr]);

    ROS_DEBUG("EL4xxx: Constructing AO object %zu", output_nr);
    std::string channel_name = "Analog output " + std::to_string(output_nr);
    outputs_[output_nr] = std::make_shared<AnalogOutput<int16_t> >(channel_name, data_ptr, nr_bits, min, max);
  }
  ROS_DEBUG("Output objects constructed");
}

}  // namespace ethercat_interface
