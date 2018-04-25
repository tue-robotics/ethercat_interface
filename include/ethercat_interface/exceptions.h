#pragma once

#include <exception>
#include <cstddef>
#include <string>

namespace ethercat_interface
{
class SocketException : public std::exception
{
  virtual const char* what() const throw()
  {
    return "No socket connection on the provided interface. "
           "If your interface is defined correctly, "
           "try excecuting the following command: "
           "sudo setcap cap_net_raw+ep on your executable";
  }
};

class NoSlaveException : public std::exception
{
  virtual const char* what() const throw()
  {
    return "No slaves found";
  }
};

class SlaveNotFoundException : public std::exception
{
public:
  SlaveNotFoundException(size_t slave_index) : slave_index_(slave_index)
  {
  }

  virtual const char* what() const throw()
  {
    return std::string("Slave " + std::to_string(slave_index_) + " not present in driver list").c_str();
  }

  size_t slave_index_;
};

class WriteException : public std::exception
{
  virtual const char* what() const throw()
  {
    return "Could not write to ethercat interface";
  }
};

class ReadException : public std::exception
{
  virtual const char* what() const throw()
  {
    return "Could not read from ethercat interface";
  }
};

}  // namespace ethercat_interface
