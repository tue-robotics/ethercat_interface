#include "ethercat_interface/ethercat_driver.h"

std::shared_ptr<ReadInterface> EtherCatDriver::getInput(unsigned int channel)
{
    ROS_INFO("Getting input on channel %d of inputs size %d", (int) channel, (int) inputs_.size());
    if (inputs_.find(channel) == inputs_.end())
    {
        throw std::runtime_error("Input " + std::to_string(channel) + " not present in " + name_ + " driver");
    }
    return inputs_[channel];
}

std::shared_ptr<WriteInterface> EtherCatDriver::getOutput(unsigned int channel)
{
    if (outputs_.find(channel) == outputs_.end())
    {
        throw std::runtime_error("Output " + std::to_string(channel) + " not present in " + name_ + " driver");
    }
    return outputs_[channel];
}

std::ostream &operator<<(std::ostream& strm, const EtherCatDriver& obj)
{
    strm << "\n\n" << obj.name_;

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
