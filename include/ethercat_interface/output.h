#pragma once

#include <memory>
#include "io.h"

namespace ethercat_interface
{
/**
 * @brief The WriteInterface class provides a base class for write interfaces
 */
class Output : public IO
{
public:
  /**
   * @brief WriteInterface Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   */
  Output(std::string name);

  /**
   * @brief write converts the provided value to the correct data type,
   * and writes it to the correct memory location such that it will be
   * send to the ethercat slaves.
   * @param value value to write
   * @return success
   */
  virtual bool write(double value) = 0;

};  // End of class WriteInterface

typedef std::shared_ptr<Output> OutputPtr;

}  // namespace ethercat_interface
