#ifndef IO_INTERFACES_H
#define IO_INTERFACES_H

#include <memory>
#include <ostream>

#include "ethercat_interface/ethercat_includes.h"


class IOInterface
{

public:
    IOInterface(std::string name) : name_(name){}

    ~IOInterface(){}

    friend std::ostream &operator<<(std::ostream&, const IOInterface&);

private:
    std::string name_;

};  // End of class IOInterface

#endif // IO_INTERFACES_H
