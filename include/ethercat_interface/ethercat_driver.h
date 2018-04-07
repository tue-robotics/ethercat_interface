#ifndef ETHERCAT_DRIVER_H
#define ETHERCAT_DRIVER_H

#include <memory>
#include <ros/console.h>
#include <vector>
#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/read_interface.h"
#include "ethercat_interface/write_interface.h"

/**
 * @brief The EtherCatDriver class provides a base class for EtherCAT drivers
 */
class EtherCatDriver
{

public:

    /**
     * @brief EtherCatDriver Constructor
     * @param slave pointer to the slave location
     */
    EtherCatDriver(ec_slavet *slave)
    {
        ec_slave_ = slave;
    }

    /**
     * @brief getChannel Returns an IO interface to one of the channels
     * @param channel represents the channel index
     * @return shared pointer to IOInterface
     */
    std::shared_ptr<ReadInterface> getInput(unsigned int channel)
    {
        ROS_INFO("Getting input on channel %d of inputs size %d", (int) channel, (int) inputs_.size());
        return inputs_[channel];
    }

    /**
     * @brief getChannel Returns an IO interface to one of the channels
     * @param channel represents the channel index
     * @return shared pointer to IOInterface
     */
    std::shared_ptr<WriteInterface> getOutput(unsigned int channel)
    {
        return outputs_[channel];
    }

protected:

    ec_slavet *ec_slave_;

    std::map<size_t, std::shared_ptr<ReadInterface> > inputs_;
    std::map<size_t, std::shared_ptr<WriteInterface> > outputs_;

};

#endif // ETHERCAT_DRIVER_H
