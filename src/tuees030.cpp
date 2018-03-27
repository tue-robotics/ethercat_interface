#include "./tuees030.h"

TUeES030::TUeES030(ec_slavet *slave) : EtherCatDriver(slave)
{
    // ToDo: resize to correct number of channels!
    channels_.resize(50);

    // Input channels
    in_tueEthercatMemoryt* input_struct_ptr = (in_tueEthercatMemoryt*)(ec_slave_->inputs);
    // ToDo: mstate1
//    channels_[1] = std::make_shared<Encoder>(&input_struct_ptr->encoder_1);

    // Output channels


}
