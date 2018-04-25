#ifndef TUEEC010_H
#define TUEEC010_H

#include "ethercat_interface/driver.h"

/**
 * @brief The TUeEC010 class is a driver for the TUeEC010 EtherCAT couplers
 * The raison d'etre for this driver is to avoid warnings at startup.
 */
class TUeEC010 : public Driver
{
public:
  /**
   * Constructor
   */
  using Driver::Driver;

};  // End of class TUeEC010

#endif  // TUEEC010_H
