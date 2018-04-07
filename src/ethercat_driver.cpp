#include "ethercat_interface/ethercat_driver.h"

std::string EtherCatDriver::getStringRep()
{
    std::string result = name_;

    if (!inputs_.empty())
    {
        result += "\nInputs:";
        for (auto& kv : inputs_)
        {
            result += "\n\t" + std::to_string(kv.first) + ": " + kv.second->getStringRep();
        }
    }

    if (!outputs_.empty())
    {
        result += "\nOutputs:";
        for (auto& kv : outputs_)
        {
            result += "\n\t" + std::to_string(kv.first) + ": " + kv.second->getStringRep();
        }
    }

    return result;
}
