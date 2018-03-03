#ifndef EL4XXX_H
#define EL4XXX_H

#include <memory>
#include <vector>
#include "ethercat_interface/io_interfaces.h"
#include "ethercat_interface/slave_driver.h"


class EL4xxx: public SlaveDriver
{
public:
    EL4xxx(ec_slavet *slave, unsigned int nr_channels,
           unsigned int nr_bits, double min, double max);

private:
    std::vector<std::shared_ptr<AO> > channels_;

};


class EL4132: public EL4xxx
{
public:
    EL4132(ec_slavet *slave);

};

#endif // EL4XXX_H
