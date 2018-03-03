#include "ethercat_interface/io_interfaces.h"
#include "ethercat_interface/ethercat_includes.h"

AO::AO(unsigned int nr_bits, double min, double max):
    nr_bits_(nr_bits), min_(min), max_(max)
{

}

bool AO::write(double value)
{
    // Convert voltage setpoint to dac value,
    // Last one and first three bits not used
    // ToDo: double check!!!
    int16_t dac_setpoint = value * (1 << (nr_bits_-1)) / (max_ - min_);

}

double AO::read()
{

}


