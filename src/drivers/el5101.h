#pragma once

#include "ethercat_interface/driver.h"
#include "../io/encoder.h"

namespace ethercat_interface
{
/**
 * @brief The EL5101 class driver for Beckhoff encoder slaves
 */
class EL5101 : public Driver
{
public:
  /**
   * @brief EL5101 Constructor
   * @param name name of this slave
   * @param slave pointer to the slave location
   */
  EL5101(ec_slavet* slave);
};

}  // namespace ethercat_interface
