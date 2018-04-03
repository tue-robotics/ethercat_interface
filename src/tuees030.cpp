#include "./tuees030.h"
#include "encoder.h"

TUeES030::TUeES030(ec_slavet *slave) : EtherCatDriver(slave)
{
    // ToDo: resize to correct number of channels!
    inputs_.resize(50);

    // Input channels
    in_tueEthercatMemoryt* input_struct_ptr = (in_tueEthercatMemoryt*)(ec_slave_->inputs);
    // ToDo: mstate1
    inputs_[1] = std::make_shared<Encoder<uint32> >(&input_struct_ptr->encoder_1);
    inputs_[2] = std::make_shared<Encoder<uint32> >(&input_struct_ptr->encoder_2);
    inputs_[3] = std::make_shared<Encoder<uint32> >(&input_struct_ptr->encoder_3);

    // Output channels


}
