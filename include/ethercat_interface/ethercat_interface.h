#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#include <memory>
#include "ethercat_interface/ethercat_driver.h"

/**
 * @brief The EthercatInterface class. Sets up the EtherCAT drivers, and exposes the IO interfaces.
 */
class EthercatInterface
{
public:

    /**
     * @brief EthercatInterface: constructor
     */
    EthercatInterface();

    /**
     * @brief ~EthercatInterface: destructor
     */
    ~EthercatInterface();

    // ToDo: move to constructor
    bool initialize(const std::string& ifname);

    // ToDo: statecheck

    // ToDo: deprecate
//    std::shared_ptr<IOInterface> getInterface(unsigned int slave, unsigned int channel);

    /**
     * @brief getSlave Returns reference to a slave
     * @param slave: slave index
     * @return reference to EtherCat driver
     */
    EtherCatDriver& getSlave(unsigned int slave);

    /**
     * @brief sendAll Sends all data over to the Ethernet device
     */
    void sendAll();

    /**
     * @brief receiveAll Reads all data from the Ethernet device
     */
    void receiveAll();

private:

    /**
     * @brief constructDrivers Constructs the driver for each slave
     * @return bool indicating success or failure
     */
    bool constructDrivers();

private:

    std::map<size_t, std::shared_ptr<EtherCatDriver> > drivers_;
    bool pdo_transfer_active_;
    volatile int wkc_, expected_wkc_;  // ToDo: correct usage of volatile?
    char IOmap_[4096];
};

#endif // ETHERCAT_INTERFACE_H
