#include "ethercat_interface/ethercat_driver.h"

std::ostream &operator<<(std::ostream& strm, const EtherCatDriver& obj)
{
    strm << obj.name_;

    if (!obj.inputs_.empty())
    {
        strm << "\nInputs:";
        for (auto& kv : obj.inputs_)
        {
            strm << "\n\t" << std::to_string(kv.first) << ": " << *kv.second;
        }
    }

    if (!obj.outputs_.empty())
    {
        strm << "\nOutputs:";
        for (auto& kv : obj.outputs_)
        {
            strm << "\n\t" << std::to_string(kv.first) << ": " << *kv.second;
        }
    }

    return strm;
}
