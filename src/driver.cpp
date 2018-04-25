#include "ethercat_interface/driver.h"

namespace ethercat_interface
{
Driver::Driver(ec_slavet* slave) : slave_(slave)
{
}

InputPtr Driver::getInput(size_t channel_index)
{
  ROS_INFO("Getting input on channel %d of inputs size %d", (int)channel_index, (int)inputs_.size());
  if (inputs_.find(channel_index) == inputs_.end())
  {
    throw std::runtime_error("Input " + std::to_string(channel_index) + " not present in " + slave_->name + " driver");
  }
  return inputs_[channel_index];
}

OutputPtr Driver::getOutput(size_t channel_index)
{
  if (outputs_.find(channel_index) == outputs_.end())
  {
    throw std::runtime_error("Output " + std::to_string(channel_index) + " not present in " + slave_->name + " driver");
  }
  return outputs_[channel_index];
}

std::ostream& operator<<(std::ostream& strm, const Driver& obj)
{
  strm << "\n\n" << obj.slave_->name;

  if (!obj.inputs_.empty())
  {
    strm << "\n\n\tInputs:";
    for (auto& kv : obj.inputs_)
    {
      strm << "\n\t\t" << std::to_string(kv.first) << ": " << *kv.second;
    }
  }

  if (!obj.outputs_.empty())
  {
    strm << "\n\n\tOutputs:";
    for (auto& kv : obj.outputs_)
    {
      strm << "\n\t\t" << std::to_string(kv.first) << ": " << *kv.second;
    }
  }

  strm << "\n";

  return strm;
}
}  // namespace ethercat_interface
