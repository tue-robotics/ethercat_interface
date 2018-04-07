#ifndef READ_INTERFACE_H
#define READ_INTERFACE_H

#include <memory>

#include "io_interfaces.h"
#include "ethercat_interface/ethercat_includes.h"



class ReadInterface : public IOInterface
{
public:

    ReadInterface(std::string name) : IOInterface(name){}

    virtual double read() = 0;

};  // End of class ReadInterface

#endif // READ_INTERFACE_H
