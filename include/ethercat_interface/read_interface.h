#ifndef READ_INTERFACE_H
#define READ_INTERFACE_H

#include "io_interfaces.h"

/**
 * @brief The ReadInterface class provides a base class for read interfaces
 */
class ReadInterface : public IOInterface
{
public:

    /**
     * @brief ReadInterface Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     */
    ReadInterface(std::string name) : IOInterface(name){}

    /**
     * @brief read reads the value from the ethercat input memory
     * location and converts it to a double
     * @return double representing the value of this input
     */
    virtual double read() = 0;

};  // End of class ReadInterface

#endif // READ_INTERFACE_H
