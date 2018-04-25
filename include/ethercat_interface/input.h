#pragma once

#include "io.h"
#include <memory>

namespace ethercat_interface
{
/**
 * @brief The Input class provides a base class for an input interface of a driver
 */
class Input : public IO
{
public:
  /**
   * @brief ReadInterface Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   */
  Input(std::string name);

  /**
   * @brief read reads the value from the ethercat input memory
   * location and converts it to a double
   * @return double representing the value of this input
   */
  virtual double read() = 0;

};  // End of class ReadInterface

typedef std::shared_ptr<Input> InputPtr;

}  // namespace ethercat_interface
