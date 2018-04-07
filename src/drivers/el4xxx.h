#ifndef EL4XXX_H
#define EL4XXX_H

#include "ethercat_interface/ethercat_driver.h"
#include "ethercat_interface/write_interface.h"

/**
 * @brief The EL4xxx class base class for drivers for Beckhoff Analog outputs
 * The three xxx indicators mean
 * i) size:
 *     0: 12 bit
 *     1: 16 bit
 * ii) range:
 *     0: 0 ... 10 V
 *     1: 0 ... 20 mA
 *     2: 4 ... 20 mA
 *     3: +/- 10 V
 * iii) nr channels
 */
class EL4xxx: public EtherCatDriver
{
public:

    /**
     * @brief EL4xxx Constructor
     * @param name name of this slave
     * @param slave memory location of the internal struct
     * @param nr_channels nr of analog output channels
     * @param nr_bits size of the integers
     * @param min minimum output value
     * @param max maximum output value
     */
    EL4xxx(std::string name, ec_slavet *slave, unsigned int nr_channels,
           unsigned int nr_bits, double min, double max);

};  // End of class EL4xxx


/**
 * @brief The EL4132 class AO driver for 2-channel, 16-bit, +/- 10 V Beckhoff slave
 */
class EL4132: public EL4xxx
{
public:
    EL4132(std::string name, ec_slavet *slave) :
        EL4xxx(name, slave, 2, 16, -10.0, 10.0){}
};  // End of class EL4132


#endif // EL4XXX_H
