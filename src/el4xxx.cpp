#include "ethercat_interface/el4xxx.h"

// ToDo: remove
#include <ros/console.h>

EL4xxx::EL4xxx(ec_slavet *slave, unsigned int nr_channels,
               unsigned int nr_bits, double min, double max) : SlaveDriver(slave)
{
    ROS_INFO("Constructing EL4xxx");
    channels_.resize(nr_channels);
    // ToDo: make nice
//    for (auto& it : channels_)
//    {
//        int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2*output_nr])
//        it = std::make_shared<AO>(nr_bits, min, max);
//    }

    ROS_INFO("Constructing output objects");
    for (unsigned int output_nr = 0; output_nr < nr_channels; output_nr++)
    {
        ROS_INFO("Getting data ptr");
        // 16 bit int vs 8 bit char
        int16_t *data_ptr = (int16_t *)&(slave->outputs[2* output_nr]);
        printf("EL4XXX: %p\n", (void*)data_ptr);
//        *data_ptr = (int16_t) 0x3FFF;
        ROS_INFO("Constructing AO object");
        channels_[output_nr] = std::make_shared<AO>(data_ptr, nr_bits, min, max);
    }
    ROS_INFO("Output objects constructed");

}

EL4132::EL4132(ec_slavet *slave) :
    EL4xxx(slave, 2, 16, -10.0, 10.0)
{
    ROS_INFO("EL4132 constructor");
}
