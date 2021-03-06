#pragma once

#include <memory>
#include "ethercat_interface/driver.h"

namespace ethercat_interface
{
/**
 * @brief The EthercatInterface class. Sets up the EtherCAT drivers, and exposes the IO interfaces.
 */
class Interface
{
public:
  /**
   * @brief EthercatInterface: constructor
   */
  Interface(const std::string& interface_name, const std::vector<std::string>& white_list = { "EK1100", "TUeEC010" });

  /**
   * @brief ~EthercatInterface: destructor
   */
  ~Interface();

  // ToDo: statecheck

  /**
   * @brief getSlave Returns reference to a slave
   * @param slave: slave index
   * @return reference to EtherCat driver
   */
  Driver& getSlave(size_t slave_index);

  /**
   * @brief read Reads all data from the Ethernet device
   */
  void read();

  /**
   * @brief write Sends all data over to the Ethernet device
   */
  void write();

private:
  /**
   * @brief constructDrivers Constructs the driver for each slave
   */
  void constructDrivers(const std::vector<std::string>& white_list);

private:
  std::map<size_t, std::shared_ptr<Driver> > drivers_;
  bool pdo_transfer_active_ = false;
  volatile int wkc_ = 0;
  volatile int expected_wkc_ = 0;  // ToDo: correct usage of volatile?
  char IOmap_[4096];
};

typedef std::shared_ptr<Interface> InterfacePtr;
}  // namespace ethercat_interface
