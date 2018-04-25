#ifndef EL5101_H
#define EL5101_H

#include "ethercat_interface/ethercat_driver.h"
#include "ethercat_interface/io_interfaces.h"

/**
 * @brief The EL5101 class driver for Beckhoff encoder slaves
 */
class EL5101: public EtherCatDriver
{
public:

    /**
     * @brief EL5101 Constructor
     * @param slave pointer to the slave location
     */
    EL5101(ec_slavet *slave);

private:
    std::shared_ptr<Encoder> encoder_;

    typedef struct PACKED
    {
      uint8 status;
      uint16 invalue;
      uint16 latch;
      uint32 frequency;
      uint16 period;
      uint16 window;
    } in_el5101t;
};

#endif // EL5101_H
