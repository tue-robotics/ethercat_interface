#include <iostream>
#include "ethercat_interface/io.h"

namespace ethercat_interface
{
std::ostream& operator<<(std::ostream& strm, const IO& obj)
{
  strm << obj.name_;
  return strm;
}

}  // namespace ethercat_interface
