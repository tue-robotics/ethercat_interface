#ifndef ETHERCAT_DRIVER_H
#define ETHERCAT_DRIVER_H

#include <memory>
#include <ros/console.h>
#include <vector>
#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/io_interfaces.h"

/**
 * @brief The EtherCatDriver class provides a base class for EtherCAT drivers
 */
class EtherCatDriver
{

public:

    /**
     * @brief EtherCatDriver Constructor
     * @param slave pointer to the slave location
     */
    EtherCatDriver(ec_slavet *slave)
    {
        ec_slave_ = slave;
    }

    /**
     * @brief getChannel Returns an IO interface to one of the channels
     * @param channel represents the channel index
     * @return shared pointer to IOInterface
     */
    std::shared_ptr<IOInterface> getChannel(unsigned int channel)
    {
        return channels_[channel];
    }

protected:

    ec_slavet *ec_slave_;

    std::vector<std::shared_ptr<IOInterface> > channels_;

};

#endif // ETHERCAT_DRIVER_H
