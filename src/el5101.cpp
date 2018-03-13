#include <iostream>
#include "ethercat_interface/el5101.h"

EL5101::EL5101(ec_slavet *slave) : EtherCatDriver(slave)
{
    std::cout << "Constructing EL5101" <<std::endl;
    in_el5101t* input_struct_ptr = (in_el5101t*)(ec_slave_->inputs);
    uint16_t* data_ptr = &input_struct_ptr->invalue;
    encoder_ = std::make_shared<Encoder>(data_ptr);

    std::cout << "input_struct_ptr: " << input_struct_ptr << ", data_ptr: "<< data_ptr << ", Encoder: " << encoder_ << std::endl;
    channels_.push_back(encoder_);
}

//Encoder &EL5101::getChannel(unsigned int channel) {
//    std::cout << "Getting channel of encoder" << std::endl;
//    return *encoder_.get();
//}

//uint32_t EL5101::read()
////unsigned int EL5101::read()
//{
////    uint32_t &ec_slave_->inputs[1];
////    return ((in_el5101t*) (m_datap->inputs))->invalue;  // Orocos SOEM
//    return ((in_el5101t*) (ec_slave_->inputs))->invalue;  // Orocos SOEM
//}
