#ifndef IO_INTERFACES_H
#define IO_INTERFACES_H

#include "ethercat_interface/ethercat_includes.h"

/**
 * @brief The IOInterface class Base class for IO interfaces
 */
class IOInterface
{

public:
//    template <typename T>
    virtual bool write(double value) = 0; // ToDo: template this
    virtual int read() = 0; // ToDo: template this
}; // End of class IOInterface

/**
 * @brief The AO class Interface for Analog Outputs
 */
class AO: public IOInterface
{
public:
    /**
     * @brief AO Constructor
     * @param data_ptr pointer to the location where the data should be written
     * @param nr_bits number of bits used for conversion
     * @param min minimum output value (before conversion from double)
     * @param max maximum output value (before conversion from double)
     */
    AO(int16_t *data_ptr, unsigned int nr_bits, double min, double max);

    /**
     * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
     * @param value value to write
     * @return bool indicating success
     */
    bool write(double value);

    // ToDo
    int read();

private:

    // ToDo 1: is int16_t the right data type
    // ToDo 2: can we move this data_ptr_ to the base class?
    int16_t* data_ptr_;
    unsigned int nr_bits_;
    double min_, max_;
}; // End of class AO

/**
 * @brief The Encoder class Encoder interface
 */
class Encoder: public IOInterface
{
public:

    /**
     * @brief Encoder Constructor. Handles overflow, assuming raw data is a 16-bit unsigned integer
     * @param data_ptr pointer to the location where the data should be written
     */
    Encoder(uint16_t *data_ptr);

    // ToDo: remove
    bool write(double value) {}

    /**
     * @brief read reads the encoder value, keeps track of overflow and converts to an int
     * @return the current encoder position (in counts)
     */
    int read();

private:
    uint16_t* data_ptr_;

    uint16_t previous_value_;  //Value of this Encoder at the previous read action
    int revolution_overflows_ = 0; // How many times has the encoder encountered a full revolution
    int encoder_max_ = UINT16_MAX; // The largest value this encoder can report before wrapping around and to a full revolution

}; // End of class Encoder

#endif // IO_INTERFACES_H
