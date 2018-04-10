#ifndef IO_INTERFACES_H
#define IO_INTERFACES_H

#include <ostream>

/**
 * @brief The IOInterface class provides a base class for read- and write interfaces
 */
class IOInterface
{

public:

    /**
     * @brief IOInterface Constructor
     * @param name of this interface. This is convenient to indicate the
     * physical connection which this interface represents
     */
    IOInterface(std::string name) : name_(name){}

    ~IOInterface(){}

    friend std::ostream &operator<<(std::ostream&, const IOInterface&);

protected:
    std::string name_;

};  // End of class IOInterface

#endif // IO_INTERFACES_H
