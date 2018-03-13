#include "ethercat_interface/io_interfaces.h"
#include "ethercat_interface/ethercat_includes.h"

#include <iostream>

AO::AO(int16_t *data_ptr, unsigned int nr_bits, double min, double max):
    data_ptr_(data_ptr), nr_bits_(nr_bits), min_(min), max_(max)
{

}

bool AO::write(double value)
{
    // Convert voltage setpoint to dac value,
    // Last one and first three bits not used
    // ToDo: double check!!!
    int16_t dac_setpoint = value * (1 << nr_bits_) / (max_ - min_);
    std::cout << "Writing value " << value << " to " << dac_setpoint << std::endl;
    *data_ptr_ = (int16_t) dac_setpoint;

    int16_t check = *data_ptr_;
    printf("Written %i to %p\n", check, data_ptr_);
//    data_ptr_ = &dac_setpoint;
    // int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2*output_nr]);
    //  *setpoint = (int16_t) volt2dac(value);
    //  std::cout << "setpoint = volt2dac = " << volt2dac(value) << std::endl;
}

unsigned int AO::read()
{

}

Encoder::Encoder(uint16_t *data_ptr) : data_ptr_(data_ptr){}

unsigned int Encoder::read()
{
//    return 123;
//    return ((in_el5101t*) (ec_slave_->inputs))->invalue;  // Orocos SOEM
    std::cout << "data_ptr_: "<< data_ptr_ << " *data_ptr: " << *data_ptr_<< std::endl;
    return *data_ptr_;
}
