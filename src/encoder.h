#ifndef ENCODER_H
#define ENCODER_H

#include "ethercat_interface/ethercat_includes.h"
#include "ethercat_interface/read_interface.h"

/**
 * @brief The Encoder class Encoder interface
 */
class Encoder : public ReadInterface
{
public:

    /**
     * @brief Encoder Constructor. Handles overflow, assuming raw data is a 16-bit unsigned integer
     * @param data_ptr pointer to the location where the data should be written
     */
    Encoder(uint16_t *data_ptr);

    /**
     * @brief read reads the encoder value, keeps track of overflow and converts to an int
     * @return the current encoder position (in counts)
     */
    double read();

private:
    uint16_t* data_ptr_;

    uint16_t previous_value_;  //Value of this Encoder at the previous read action
    int revolution_overflows_ = 0; // How many times has the encoder encountered a full revolution
    int encoder_max_ = UINT16_MAX; // The largest value this encoder can report before wrapping around and to a full revolution

}; // End of class Encoder

#endif // ENCODER_H
