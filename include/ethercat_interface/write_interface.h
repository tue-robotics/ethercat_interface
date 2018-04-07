#ifndef WRITE_INTERFACE_H
#define WRITE_INTERFACE_H

#include <memory>

#include "ethercat_interface/ethercat_includes.h"
#include "io_interfaces.h"

class WriteInterface : public IOInterface
{
public:

    WriteInterface(std::string name) : IOInterface(name){}

    virtual bool write(double value) = 0;

};  // End of class WriteInterface

#endif // WRITE_INTERFACE_H
