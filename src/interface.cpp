#include <ros/console.h>

#include "ethercat_interface/interface.h"
#include "ethercat_interface/exceptions.h"

// List of driver implementations
#include "./drivers/el200x.h"
#include "./drivers/el4xxx.h"
#include "./drivers/el5101.h"
#include "./drivers/tuees030.h"

namespace ethercat_interface
{
Interface::Interface(const std::string& interface_name, const std::vector<std::string> white_list)
{
  // Initialise SOEM, bind socket to ifname
  // Use const_cast here because we have an old c interface
  if (!ec_init(const_cast<char*>(interface_name.c_str())))
  {
    throw SocketException();
  }
  ROS_INFO("ec_init on %s succeeded.", interface_name.c_str());

  // Find and auto-config slaves
  if (ec_config_init(false) <= 0)
  {
    throw NoSlaveException();
  }
  ROS_INFO("%d slaves found and configured.", ec_slavecount);

  char io_map[4096];
  ec_config_map(&io_map);
  ec_configdc();
  ROS_INFO("Slaves mapped, state to SAFE_OP.");

  // Wait for all slaves to reach SAFE_OP state
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ROS_INFO("segments : %d : %d %d %d %d", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
           ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

  ROS_INFO("Request operational state for all slaves");
  int expected_wkc = 0;
  expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ROS_INFO("Calculated workcounter %d", expected_wkc);
  ec_slave[0].state = EC_STATE_OPERATIONAL;

  // Send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  // request OP state for all slaves
  ec_writestate(0);

  // Wait for all slaves to reach OP state
  int chk = 40;  // ToDo: make nice
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
    for (size_t i = 1; i <= ec_slavecount; i++)  // ToDo: make nice?
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        ROS_WARN("Slave %zu State=0x%2.2x StatusCode=0x%4.4x : %s", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                 ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
  }

  constructDrivers(white_list);
}

Interface::~Interface()
{
  // Stop PDO transfer
  pdo_transfer_active_ = false;

  // Request INIT state for all slaves
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);

  // Stop SOEM, close socket
  ec_close();
}

void Interface::constructDrivers(const std::vector<std::string> white_list)
{
  for (size_t slave_index = 0; slave_index < ec_slavecount; slave_index++)
  {
    size_t ec_slave_idx = slave_index + 1;  // why ??
    ec_slavet* slave = &ec_slave[ec_slave_idx];

    ROS_INFO("Driver %zu: %s", slave_index, slave->name);

    // ToDo: classloader/driverfactory/pluginglib implementation
    DriverPtr driver;
    if (slave->name == "EL2002")
      driver = std::make_shared<EL2002>(slave);
    if (slave->name == "EL2004")
      driver = std::make_shared<EL2004>(slave);
    if (slave->name == "EL2008")
      driver = std::make_shared<EL2008>(slave);
    if (slave->name == "EL4132")
      driver = std::make_shared<EL4132>(slave);
    if (slave->name == "EL5101")
      driver = std::make_shared<EL5101>(slave);
    if (slave->name == "TUeES030")
      driver = std::make_shared<TUeES030>(slave);

    if (driver)
    {
      drivers_[slave_index] = driver;
      ROS_DEBUG_STREAM(*driver);
    }
    // If element not in white list and we did not find a valid driver
    else if (std::find(white_list.begin(), white_list.end(), slave->name) == white_list.end())
    {
      ROS_WARN("Could not find an imlementation for driver '%s'. Either implement the driver or add the driver name to"
               "the white list in order to suppress this warning message.",
               slave->name);
    }
  }
}

Driver& Interface::getSlave(size_t slave_index)
{
  ROS_INFO("Getting slave with index %zu of slaves vector of size %zu", slave_index, drivers_.size());

  if (drivers_.find(slave_index) == drivers_.end())
  {
    throw SlaveNotFoundException(slave_index);
  }

  return *drivers_[slave_index].get();
}

void Interface::read()
{
  if (pdo_transfer_active_)
  {
    wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
  }
  else
  {
    throw ReadException();
  }
}

void Interface::write()
{
  if (pdo_transfer_active_)
  {
    ec_send_processdata();
  }
  else
  {
    throw WriteException();
  }
}

}  // namespace ethercat_interface
