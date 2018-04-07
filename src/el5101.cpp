#include "./el5101.h"

EL5101::EL5101(ec_slavet *slave) : EtherCatDriver(slave)
{
    in_el5101t* input_struct_ptr = (in_el5101t*)(ec_slave_->inputs);
    uint16_t* data_ptr = &input_struct_ptr->invalue;
    encoder_ = std::make_shared<Encoder<uint16> >(data_ptr);

    inputs_[0] = encoder_;
}
