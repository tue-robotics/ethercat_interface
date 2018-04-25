#ifndef EK1100_H
#define EK1100_H

#include "ethercat_interface/driver.h"

/**
 * @brief The EK1100 class is a driver for the EK1100 EtherCAT couplers
 * The raison d'etre for this driver is to avoid warnings at startup.
 */
class EK1100 : public Driver
{
public:
  /**
   * Constructor
   */
  using Driver::Driver;

};  // End of class EK1100

#endif  // EK1100_H
