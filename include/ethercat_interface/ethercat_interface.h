#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#include <memory>
#include "ethercat_interface/ethercat_driver.h"

class EthercatInterface
{
public:

    /**
     * @brief EthercatInterface: ToDo
     */
    EthercatInterface();

    ~EthercatInterface();

    bool initialize(const std::string& ifname);

    // ToDo: statecheck

    // ToDo: deprecate
    std::shared_ptr<IOInterface> getInterface(unsigned int slave, unsigned int channel);

    EtherCatDriver& getSlave(unsigned int slave);

    void sendAll();

    void receiveAll();

private:

    bool constructDrivers();

private:

    std::vector<std::shared_ptr<EtherCatDriver> > drivers_;
    bool pdo_transfer_active_;
    volatile int wkc_, expected_wkc_;  // ToDo: correct usage of volatile?
    char IOmap_[4096];
};

#endif // ETHERCAT_INTERFACE_H
