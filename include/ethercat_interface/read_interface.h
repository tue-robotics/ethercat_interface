#ifndef READ_INTERFACE_H
#define READ_INTERFACE_H

#include "ethercat_interface/ethercat_includes.h"
#include <memory>


class ReadInterface
{
public:

    virtual double read() = 0;

};  // End of class ReadInterface

#endif // READ_INTERFACE_H
