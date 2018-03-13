#ifndef EL5101_H
#define EL5101_H

#include "ethercat_interface/ethercat_driver.h"
#include "ethercat_interface/io_interfaces.h"

class EL5101: public EtherCatDriver
{
public:
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
