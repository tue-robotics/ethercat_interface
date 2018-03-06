#ifndef IO_INTERFACES_H
#define IO_INTERFACES_H

#include "ethercat_interface/ethercat_includes.h"

class IOInterface
{

public:
//    template <typename T>
    virtual bool write(double value) = 0; // ToDo: template this
};

class AO: public IOInterface
{
public:
    AO(int16_t *data_ptr, unsigned int nr_bits, double min, double max);

    bool write(double value);

    double read();

private:

    // ToDo 1: is int16_t the right data type
    // ToDo 2: can we move this data_ptr_ to the base class?
    int16_t* data_ptr_;
    unsigned int nr_bits_;
    double min_, max_;
};

class Encoder: public IOInterface
{
public:
    Encoder(uint16_t *data_ptr);

    unsigned int read();
    bool write(double value){};

private:
    uint16_t* data_ptr_;
//    return ((in_el5101t*) (m_datap->inputs))->invalue;
}; // End of class Encoder

#endif // IO_INTERFACES_H

//int16_t EL4002::volt2dac(double value){
//  // 10V = 32767 2^15-1
//  //  0V = 16 = 2^4

//  //convert voltage setpoint to 16 bits dac value, Last one and first three bits not used
//  int16_t dac_setpoint = value*(1<<15)/10;

//  return dac_setpoint;
//}

//double EL4002::dac2volt(int16_t dac_value){
//  //convert 16 bits dac value to voltage
//  double voltage = dac_value; //TODO
//  return voltage;
//}

//void EL4002::set_output(uint8_t output_nr, double value){
//  //2 bytes per output
//  int16_t *setpoint = (int16_t *)&(ec_slave->outputs[2*output_nr]);
//  *setpoint = (int16_t) volt2dac(value);
//  std::cout << "setpoint = volt2dac = " << volt2dac(value) << std::endl;

//  std::cout << (int16_t) ec_slave->outputs[0] << " - "  << (int16_t) ec_slave->outputs[1] << " - "  << (int16_t) ec_slave->outputs[2] << " - "  << (int16_t) ec_slave->outputs[3] << std::endl;
//}

//double EL4002::get_output(uint8_t output_nr){
//  return dac2volt(ec_slave->outputs[0] & (1<<output_nr));
//}
