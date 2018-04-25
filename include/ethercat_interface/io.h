#pragma once

#include <ostream>

namespace ethercat_interface
{
/**
 * @brief The IOInterface class provides a base class for read- and write interfaces
 */
class IO
{
public:
  /**
   * @brief IOInterface Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   */
  IO(std::string name) : name_(name)
  {
  }

  ~IO()
  {
  }

  friend std::ostream& operator<<(std::ostream&, const IO&);

protected:
  std::string name_;

};  // End of class IOInterface

}  // namespace ethercat_interface
