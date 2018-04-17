#include <ros/console.h>

#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

#include "./drivers/drivers.h"


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
    for (unsigned int i = 1; i <= ec_slavecount; i++) // ToDo: make nice?
    {
        std::string name(ec_slave[i].name);
        ROS_INFO("Driver %u: %s", i, name.c_str());
        // ToDo: classloader/driverfactory/pluginglib implementation
        if (name == "EK1100")
        {
            ROS_INFO("Constructing EK1100 for i %i", i);
            drivers_[i-1] = std::make_shared<EK1100>(name, &ec_slave[i]);
        }
        else if (name == "EL2002")
        {
            ROS_INFO("Constructing EL2002 for i %i", i);
            drivers_[i-1] = std::make_shared<EL2002>(name, &ec_slave[i]);
        }
        else if (name == "EL2004")
        {
            ROS_INFO("Constructing EL2004 for i %i", i);
            drivers_[i-1] = std::make_shared<EL2004>(name, &ec_slave[i]);
        }
        else if (name == "EL2008")
        {
            ROS_INFO("Constructing EL2008 for i %i", i);
            drivers_[i-1] = std::make_shared<EL2008>(name, &ec_slave[i]);
        }
        else if (name == "EL4132")
        {
            ROS_INFO("Constructing EL4132 for i %i", i);
            drivers_[i-1] = std::make_shared<EL4132>(name, &ec_slave[i]);
        }
        else if (name == "EL5101")
        {
            ROS_INFO("Constructing EL5101 for i %i", i);
            drivers_[i-1] = std::make_shared<EL5101>(name, &ec_slave[i]);
        }
        else if (name == "TUeEC010")
        {
            ROS_INFO("Constructing TUeEC010 for i %i", i);
            drivers_[i-1] = std::make_shared<TUeEC010>(name, &ec_slave[i]);
        }
        else if (name == "TUeES030")
        {
            ROS_INFO("Constructing TUeES030 for i %i", i);
            drivers_[i-1] = std::make_shared<TUeES030>(name, &ec_slave[i]);
        }
        else
        {
            ROS_WARN("No driver for %u: %s", i, name.c_str());
        }

        // Print driver info
        if (drivers_.find(i-1) != drivers_.end())
        {
            ROS_DEBUG_STREAM(*drivers_[i-1]);
        }

    }
    return true;
}

EtherCatDriver& EthercatInterface::getSlave(unsigned int slave)
{
    ROS_INFO("Getting slave with index %d of slaves vector of size %d", (int) slave, (int) drivers_.size());

    if (drivers_.find(slave) == drivers_.end())
    {
        throw std::runtime_error("Slave " + std::to_string(slave) + " not present in driver list");
    }

    return *drivers_[slave].get();
}

void EthercatInterface::sendAll()
{
    if (pdo_transfer_active_)
    {
//        ec_slavet* slave = &ec_slave[2];
//        int16_t *value = (int16_t *)&(slave->outputs[0]);
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
