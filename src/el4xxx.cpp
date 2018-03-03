#include "ethercat_interface/el4xxx.h"

EL4xxx::EL4xxx(ec_slavet *slave, unsigned int nr_channels,
               unsigned int nr_bits, double min, double max) : SlaveDriver(slave)
{
    channels_.resize(nr_channels);
    // ToDo: make nice
    for (auto& it : channels_) {
        it = std::make_shared<AO>(nr_bits, min, max);
    }

}

EL4132::EL4132(ec_slavet *slave) :
    EL4xxx(slave, 2, 16, -10.0, 10.0)
{

}
