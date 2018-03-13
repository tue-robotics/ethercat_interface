#include <iostream>
#include "ethercat_interface/el5101.h"

#include <ros/console.h>
#include <ethercat_interface/el5101.h>

EL5101::EL5101(ec_slavet *slave) : EtherCatDriver(slave)
{
    std::cout << "Constructing EL5101" <<std::endl;
    in_el5101t* input_struct_ptr = (in_el5101t*)(ec_slave_->inputs);
    uint16_t* data_ptr = &input_struct_ptr->invalue;
    encoder_ = std::make_shared<Encoder>(data_ptr);

    std::cout << "input_struct_ptr: " << input_struct_ptr << ", data_ptr: "<< data_ptr << ", Encoder: " << encoder_ << std::endl;
    channels_.push_back(encoder_);
}

void EL5101::tempUpdate()
{
    in_el5101t* input_struct_ptr = (in_el5101t*)(ec_slave_->inputs);

    unsigned int status = input_struct_ptr->status;
    ROS_INFO_THROTTLE(1.0, "Status: %u", status);

    unsigned int invalue = input_struct_ptr->invalue;
    ROS_INFO_THROTTLE(1.0, "invalue: %u", invalue);

    unsigned int latch = input_struct_ptr->latch;
    ROS_INFO_THROTTLE(1.0, "latch: %u", latch);

    unsigned int frequency = input_struct_ptr->frequency;
    ROS_INFO_THROTTLE(1.0, "frequency: %u", frequency);

    unsigned int period = input_struct_ptr->period;
    ROS_INFO_THROTTLE(1.0, "period: %u", period);

    unsigned int window = input_struct_ptr->window;
    ROS_INFO_THROTTLE(1.0, "window: %u", window);
}

//int8 status;
//uint16 invalue;
//uint16 latch;
//uint32 frequency;
//uint16 period;
//uint16 window;
//} in_el5101t;

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
