#include "ethercat_interface/el5101.h"

EL5101::EL5101(ec_slavet *slave) : EtherCatDriver(slave)
{
    uint16_t* data_ptr = &((in_el5101t*)(ec_slave_->inputs))->invalue;
    encoder_ = std::make_shared<Encoder>(data_ptr);
}

//uint32_t EL5101::read()
//unsigned int EL5101::read()
//{
////    uint32_t &ec_slave_->inputs[1];
////    return ((in_el5101t*) (m_datap->inputs))->invalue;  // Orocos SOEM
//    return ((in_el5101t*) (ec_slave_->inputs))->invalue;  // Orocos SOEM
//}
