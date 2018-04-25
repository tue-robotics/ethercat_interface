#pragma once

#include <memory>
#include <map>
#include <ros/console.h>
#include <vector>
#include "ethercat_interface/input.h"
#include "ethercat_interface/output.h"

#include "soem/ethercattype.h"
#include "soem/nicdrv.h"
#include "soem/osal.h"
#include "soem/oshw.h"
#include "soem/ethercatmain.h"
#include "soem/ethercatdc.h"
#include "soem/ethercatcoe.h"
#include "soem/ethercatfoe.h"
#include "soem/ethercatsoe.h"
#include "soem/ethercatconfig.h"
#include "soem/ethercatprint.h"

namespace ethercat_interface
{
/**
 * @brief The Driver class provides a base class for EtherCAT drivers
 */
class Driver
{
public:
  /**
   * @brief Driver Constructor
   * @param slave pointer to the slave
   */
  Driver(ec_slavet* slave);

  /**
   * @brief getInput Returns an input based on a channel idx
   * @param channel represents the channel index
   * @return shared pointer to a ReadInterface
   */
  InputPtr getInput(size_t channel_index);

  /**
   * @brief getOutput Returns a WriteInterface to one of the channels
   * @param channel represents the channel index
   * @return shared pointer to the WriteInterface
   */
  OutputPtr getOutput(size_t channel_index);

  friend std::ostream& operator<<(std::ostream&, const Driver&);

protected:
  ec_slavet* slave_;

  std::map<size_t, InputPtr> inputs_;
  std::map<size_t, OutputPtr> outputs_;
};

typedef std::shared_ptr<Driver> DriverPtr;

}  // namespace ethercat_interface
