#include <ros/console.h>
#include "ethercat_interface/drivers.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

EthercatInterface::EthercatInterface():
    pdo_transfer_active_(false),
    wkc_(0), expected_wkc_(0)
{

}

EthercatInterface::~EthercatInterface()
{
    // Stop PDO transfer
    pdo_transfer_active_ = false;

    // Request INIT state for all slaves
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    // Stop SOEM, close socket
    ec_close();
}

bool EthercatInterface::initialize(const std::string &ifname)
{
    // Initialise SOEM, bind socket to ifname
    // Use const_cast here because we have an old c interface
    if (!ec_init(const_cast<char*>(ifname.c_str())))
    {
        throw socketerror;
    }
    ROS_INFO("ec_init on %s succeeded.", ifname.c_str());

    // Find and auto-config slaves
    if (ec_config_init(false) <= 0)
    {
        throw noslaveerror;
    }
    ROS_INFO("%d slaves found and configured.", ec_slavecount);

    ec_config_map(&IOmap_);
    ec_configdc();
    ROS_INFO("Slaves mapped, state to SAFE_OP.");

    // Wait for all slaves to reach SAFE_OP state
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    ROS_INFO("segments : %d : %d %d %d %d", ec_group[0].nsegments,
            ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
            ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

    ROS_INFO("Request operational state for all slaves");
    expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    ROS_INFO("Calculated workcounter %d", expected_wkc_);
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // Send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // request OP state for all slaves
    ec_writestate(0);

    // Wait for all slaves to reach OP state
    int chk = 40; // ToDo: make nice
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        ROS_INFO("Operational state reached for all slaves.");
        pdo_transfer_active_ = true;
    }
    else
    {
        ROS_WARN("Not all slaves reached operational state.");
        ec_readstate();
        for (unsigned int i = 1; i <= ec_slavecount; i++) // ToDo: make nice?
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                ROS_WARN("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s", i,
                         ec_slave[i].state, ec_slave[i].ALstatuscode,
                         ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }

    if (!constructDrivers())
    {
        return false;
    }

    return true;

}

bool EthercatInterface::constructDrivers()
{
    drivers_.resize(ec_slavecount);
    for (unsigned int i = 1; i <= ec_slavecount; i++) // ToDo: make nice?
    {
        std::string name(ec_slave[i].name);
        ROS_INFO("Driver %u: %s", i, name.c_str());
        // ToDo: classloader/driverfactory/pluginglib implementation
        if (name == "EK1100")
        {
//            ec_slavet *slave = &ec_slave[i];
            drivers_[i-1] = std::make_shared<EK1100>(&ec_slave[i]);
        }
        else if (name == "EL4132")
        {
            ROS_INFO("Creating EL4132 driver");
//            std::make_shared<EL4132>(&ec_slave[i]);
            drivers_[i-1] = std::make_shared<EL4132>(&ec_slave[i]);
            ROS_INFO("Created EL4132 driver");
        }
        else
        {
            ROS_WARN("No driver for %u: %s", i, name.c_str());
        }

    }
    return true;
}

std::shared_ptr<IOInterface> EthercatInterface::getInterface(unsigned int slave, unsigned int channel){
    return drivers_[slave]->getChannel(channel);
}

void EthercatInterface::sendAll()
{
    if (pdo_transfer_active_)
    {
        ec_slavet* slave = &ec_slave[2];
        int16_t *value = (int16_t *)&(slave->outputs[0]);
//        printf("sendall address: %p, value: %i\n", ec_slave[2].outputs, *value);
        ec_send_processdata();
    }

}

void EthercatInterface::receiveAll()
{
    if (pdo_transfer_active_)
    {
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
    }

}
