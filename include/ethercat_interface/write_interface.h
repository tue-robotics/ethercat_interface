#ifndef WRITE_INTERFACE_H
#define WRITE_INTERFACE_H

#include "io_interfaces.h"

/**
 * @brief The WriteInterface class provides a base class for write interfaces
 */
class WriteInterface : public IOInterface
{
public:

    /**
     * @brief WriteInterface Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     */
    WriteInterface(std::string name) : IOInterface(name){}

    /**
     * @brief write converts the provided value to the correct data type,
     * and writes it to the correct memory location such that it will be
     * send to the ethercat slaves.
     * @param value value to write
     * @return success
     */
    virtual bool write(double value) = 0;

};  // End of class WriteInterface

#endif // WRITE_INTERFACE_H
