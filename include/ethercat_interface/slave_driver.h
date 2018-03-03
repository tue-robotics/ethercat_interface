#ifndef SLAVE_DRIVER_H
#define SLAVE_DRIVER_H

#include "ethercat_interface/ethercat_includes.h"

class SlaveDriver
{

public:
    SlaveDriver(ec_slavet *slave)
    {
        ec_slave_ = slave;
    }

protected:
    ec_slavet *ec_slave_;

};

#endif // SLAVE_DRIVER_H
