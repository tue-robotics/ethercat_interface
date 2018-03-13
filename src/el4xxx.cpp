#include "ethercat_interface/el4xxx.h"

EL4xxx::EL4xxx(ec_slavet *slave, unsigned int nr_channels,
               unsigned int nr_bits, double min, double max) : EtherCatDriver(slave)
{
    ROS_INFO("Constructing EL4xxx with %u channels, %u bits, range [%.2f, %.2f]",
             nr_channels, nr_bits, min, max);
    channels_.resize(nr_channels);

    ROS_DEBUG("EL4xxx: Constructing output objects");
    for (unsigned int output_nr = 0; output_nr < nr_channels; output_nr++)
    {
        ROS_DEBUG("EL4xxx: Getting data ptr %u", output_nr);
        // 16 bit int vs 8 bit char
        int16_t *data_ptr = (int16_t *)&(slave->outputs[2* output_nr]);

        ROS_DEBUG("EL4xxx: Constructing AO object %u", output_nr);
        channels_[output_nr] = std::make_shared<AO>(data_ptr, nr_bits, min, max);
    }
    ROS_DEBUG("Output objects constructed");

}

IOInterface& EL4xxx::getChannel(unsigned int channel)
{
    return *channels_[channel].get();
}
