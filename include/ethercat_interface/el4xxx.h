#ifndef EL4XXX_H
#define EL4XXX_H

#include <memory>
#include <vector>
#include "ethercat_interface/io_interfaces.h"
#include "ethercat_interface/slave_driver.h"


// ToDo: use template
class EL4xxx: public SlaveDriver
{
public:
    EL4xxx(ec_slavet *slave, unsigned int nr_channels,
           unsigned int nr_bits, double min, double max);

private:
    // ToDo: generalize (don't hardcode this 2)
    typedef struct PACKED
    {
        int16_t values[2];
    } out_el4xxxt;

};


class EL4132: public EL4xxx
{
public:
    EL4132(ec_slavet *slave);
};

#endif // EL4XXX_H
