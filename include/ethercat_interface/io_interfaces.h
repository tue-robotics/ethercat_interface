#ifndef IO_INTERFACES_H
#define IO_INTERFACES_H

#include "ethercat_interface/ethercat_includes.h"
#include <memory>

class IOInterface
{

public:
    IOInterface(std::string name) : name_(name){}

    ~IOInterface(){}

    std::string getStringRep(){return name_;}

private:
    std::string name_;

};  // End of class IOInterface

#endif // IO_INTERFACES_H
