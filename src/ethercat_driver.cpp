#include "ethercat_interface/ethercat_driver.h"

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
