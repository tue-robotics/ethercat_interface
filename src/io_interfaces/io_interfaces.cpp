#include <iostream>
#include "ethercat_interface/io_interfaces.h"

std::ostream &operator<<(std::ostream& strm, const IOInterface& obj)
{
    strm << obj.name_;
    return strm;
}
