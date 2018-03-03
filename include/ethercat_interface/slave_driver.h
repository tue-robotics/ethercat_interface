#ifndef SLAVE_DRIVER_H
#define SLAVE_DRIVER_H

#include <memory>
#include <vector>
#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/io_interfaces.h"

class SlaveDriver
{

public:
    SlaveDriver(ec_slavet *slave)
    {
        ec_slave_ = slave;
    }

    std::shared_ptr<IOInterface> getChannel(unsigned int channel)
    {
        return channels_[channel];
    }

protected:
    ec_slavet *ec_slave_;

    std::vector<std::shared_ptr<IOInterface> > channels_;

};

#endif // SLAVE_DRIVER_H
