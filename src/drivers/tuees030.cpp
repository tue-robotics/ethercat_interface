#include <stdexcept>
#include "ethercat_interface/input.h"
#include "ethercat_interface/output.h"
#include "./tuees030.h"
#include "../io/analog_input.h"
#include "../io/analog_output.h"
#include "../io/digital_input.h"
#include "../io/digital_output.h"
#include "../io/encoder.h"

namespace ethercat_interface
{
/**
 * @brief The TUeES030Input class input class specific for TUeES030
 */
template <typename T>
class TUeES030Input : public Input
{
public:
  /**
   * @brief TUeES030Input Constructor.
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   * @param scale: double: the raw input value is multiplied by the scale before it is returned
   */
  TUeES030Input(std::string name, T* data_ptr, double scale) : data_ptr_(data_ptr), scale_(scale), Input(name)
  {
  }

  /**
   * @brief read reads the analog input
   * @return the analog input value
   */
  double read()
  {
    // Read the integer value from the memory
    T int_value = *data_ptr_;

    // Convert to float
    double float_value = static_cast<double>(int_value);

    // Scale
    double return_value = scale_ * float_value;

    ROS_DEBUG_THROTTLE(1.0, "TUeES030Input: %.1f --> %.1f", float_value, return_value);

    return return_value;
  }

private:
  T* data_ptr_;
  double scale_;

};  // End of class TUeES030Input

/**
 * @brief The MotorCommand class Interface for TUEes030 MotorCommands. This is specific
 * for the TUes030 driver
 */
class MotorCommand : public Output
{
public:
  /**
   * @brief MotorCommand Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   * @param motor_id: identifies for which motor this command is intended (1, 2 or 3)
   */
  MotorCommand(std::string name, std::bitset<8>* data_ptr, unsigned int motor_id)
    : data_ptr_(data_ptr), Output(name), position_(2)
  {
    // Check input
    if (motor_id != 1 && motor_id != 2 && motor_id != 3)
    {
      throw std::invalid_argument("Motor id must be 1, 2 or 3");
    }

    // Reset the data
    data_ptr_->reset();

    // ToDo: make nice (motor 1: 00, motor 2: 01, motor 3: 10)
    // Set first bit
    if (motor_id == 2)
    {
      data_ptr_->set(0, true);
    }

    // Set second bit
    if (motor_id == 3)
    {
      data_ptr_->set(1, true);
    }

  }  // End of constructor

  /**
   * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus
   * @param value value to write
   * @return bool indicating success
   */
  bool write(double value)
  {
    // Write the value
    data_ptr_->set(position_, bool(value));
    return true;
  }

private:
  std::bitset<8>* data_ptr_;
  unsigned int position_;

};  // End of class MotorCommand

/**
 * @brief The ES030Output class Interface for Outputs for the TUeES030. This converts
 * the input differently than a AnalogOutput, i.e., by representing each mV or mA as a bit
 */
class ES030Output : public Output
{
public:
  /**
   * @brief ES030Output Constructor
   * @param name of this interface. This is convenient to indicate the
   * physical connection which this interface represents
   * @param data_ptr pointer to the location where the data should be written
   * @param min: mimimum value (V or A)
   * @param max: maximum value (V or A)
   */
  ES030Output(std::string name, int16* data_ptr, double min, double max)
    : data_ptr_(data_ptr), min_(min), max_(max), Output(name)
  {
    *data_ptr_ = 0;
    write(0.0);
  }

  /**
   * @brief write Writes the value in the EtherCAT memory before it is send over the EtherCAT bus.
   * @param value value to write in V or A
   * @return bool indicating success
   */
  bool write(double value)
  {
    // Check/clip input
    if (value > max_)
    {
      ROS_WARN("%s value %.2f exceeds maximum %.2f, clipping", name_.c_str(), value, max_);
      value = max_;
    }
    else if (value < min_)
    {
      ROS_WARN("%s value %.2f exceeds minimum %.2f, clipping", name_.c_str(), value, min_);
      value = min_;
    }

    // Convert to mA
    double m_value = 1000.0 * value;

    // Convert to integer
    int i_value = static_cast<int>(m_value);

    // Debug info
    ROS_DEBUG_THROTTLE(1.0, "%s: Writing %.2f [V or A], %f [mV or mA], %i bits", name_.c_str(), value, m_value,
                       i_value);

    // Write the value to the designated memory location
    *data_ptr_ = i_value;

    return true;
  }

private:
  int16* data_ptr_;
  double min_, max_;

};  // End of class AO

/**
 * @brief TUeES030::TUeES030 Driver for a TUeES030 Ethercat slave
 * @param name
 * @param slave
 */
TUeES030::TUeES030(ec_slavet* slave) : Driver(slave)
{
  double current_scale = 0.001;  // ToDo: doublecheck

  // Input channels
  in_tueEthercatMemoryt* input_struct_ptr = (in_tueEthercatMemoryt*)(slave_->inputs);
  // ToDo: mstate1 [0]
  inputs_[1] = std::make_shared<Encoder<uint32> >("Encoder1", &input_struct_ptr->encoder_1);
  inputs_[2] = std::make_shared<TUeES030Input<int16> >("Current1", &input_struct_ptr->current_1, current_scale);
  // ToDo: mstate2 [3]
  inputs_[4] = std::make_shared<Encoder<uint32> >("Encoder2", &input_struct_ptr->encoder_2);
  inputs_[5] = std::make_shared<TUeES030Input<int16> >("Current2", &input_struct_ptr->current_2, current_scale);
  // ToDo: mstate3 [6]
  inputs_[7] = std::make_shared<Encoder<uint32> >("Encoder3", &input_struct_ptr->encoder_3);
  inputs_[8] = std::make_shared<TUeES030Input<int16> >("Current3", &input_struct_ptr->current_3, current_scale);

  // Digital inputs
  digital_in_t* digital_in = &input_struct_ptr->digital_in;  // ToDo: check order
  std::bitset<8>* line_input_ptr = (std::bitset<8>*)&digital_in->line;
  inputs_[9] = std::make_shared<DigitalInput<8> >("SpareDi1", line_input_ptr, 0);
  inputs_[10] = std::make_shared<DigitalInput<8> >("SpareDi2", line_input_ptr, 1);
  inputs_[11] = std::make_shared<DigitalInput<8> >("SpareDi3", line_input_ptr, 2);
  inputs_[12] = std::make_shared<DigitalInput<8> >("SpareDi4", line_input_ptr, 3);
  //    inputs_[13] = std::make_shared<DigitalInput<8> >("Reserved1", line_input_ptr, 4);  // Do not expose?!
  //    inputs_[14] = std::make_shared<DigitalInput<8> >("Reserved2", line_input_ptr, 5);  // Do not expose?!
  //    inputs_[15] = std::make_shared<DigitalInput<8> >("Reserved3", line_input_ptr, 6);  // Do not expose?!
  inputs_[16] = std::make_shared<DigitalInput<8> >("PowerStatus", line_input_ptr, 7);

  // Caliphers
  double calipher_scale = 0.01 * 0.001;  // 0.01 mm/bit * 0.001 m / mm
  inputs_[18] = std::make_shared<TUeES030Input<uint16> >("Calipher1", &input_struct_ptr->calipher_1, calipher_scale);
  inputs_[19] = std::make_shared<TUeES030Input<uint16> >("Calipher2", &input_struct_ptr->calipher_2, calipher_scale);

  // Force inputs
  double ai_conversion_scale = 4095.0 / 3.3;
  inputs_[20] = std::make_shared<TUeES030Input<uint16> >("Force1", &input_struct_ptr->force_1, ai_conversion_scale);
  inputs_[21] = std::make_shared<TUeES030Input<uint16> >("Force2", &input_struct_ptr->force_2, ai_conversion_scale);
  inputs_[22] = std::make_shared<TUeES030Input<uint16> >("Force3", &input_struct_ptr->force_3, ai_conversion_scale);

  // Position inputs
  inputs_[23] = std::make_shared<TUeES030Input<uint16> >("Pos1", &input_struct_ptr->position_1, ai_conversion_scale);
  inputs_[24] = std::make_shared<TUeES030Input<uint16> >("Pos2", &input_struct_ptr->position_2, ai_conversion_scale);
  inputs_[25] = std::make_shared<TUeES030Input<uint16> >("Pos3", &input_struct_ptr->position_3, ai_conversion_scale);

  // Spare inputs
  inputs_[26] =
      std::make_shared<AnalogInput<uint16> >("SpareAI1", &input_struct_ptr->spare_ai_1);  // ToDo: other conversion?
  inputs_[27] =
      std::make_shared<AnalogInput<uint16> >("SpareAI2", &input_struct_ptr->spare_ai_2);  // ToDo: other conversion?
  inputs_[28] = std::make_shared<TUeES030Input<uint16> >("Linevoltage", &input_struct_ptr->linevoltage, 0.01);

  //    uint8       mstate1;			// motor state 1
  //    uint32      encoder_1;			// Encoder 1
  //    uint32      timestamp1;			// Time stamp encoder 1 (Only changes, if encoder changes)
  //    int16       velocity1;			// Velocity encoder 1; 1 bit=0.1 rad/s; depending on parameters on slave
  //    int16       current_1;			// current on PWM 1 (1 bit = 1 mA)
  //    uint8       mstate2;			// motor state 2
  //    uint32      encoder_2;			// Encoder 2
  //    uint32      timestamp2;			// Time stamp encoder 2 (Only changes, if encoder changes)
  //    int16       velocity2;			// Velocity encoder 2; 1 bit=0.1 rad/s; depending on parameters on slave
  //    int16       current_2;			// current on PWM 2 (1 bit = 1 mA)
  //    uint8       mstate3;                        // motor state 3
  //    uint32      encoder_3;			// Encoder 3
  //    uint32      timestamp3;			// Time stamp encoder 3 (Only changes, if encoder changes)
  //    int16       velocity3;			// Velocity encoder 3; 1 bit=0.1 rad/s; depending on parameters on slave
  //    int16       current_3;			// current on PWM 2 (1 bit = 1 mA)
  //    digital_in_t digital_in;                    // digital input 8 bits
  //    uint16      calipher_1;			// calipher 1 (1 bit = 0.01 mm)
  //    uint16      calipher_2;			// calipher 2 (1 bit = 0.01 mm)
  //    uint16      force_1;			// Analog ADC value of force sensor input 1
  //    uint16      force_2;			// Analog ADC value of force sensor input 2
  //    uint16      force_3;			// Analog ADC value of force sensor input 3
  //    uint16      position_1;			// Analog ADC value of position sensor 1
  //    uint16      position_2;			// Analog ADC value of position sensor 2
  //    uint16      position_3;			// Analog ADC value of position sensor 3
  //    uint16      spare_ai_1;			// Spare analog in 1
  //    uint16      spare_ai_2;			// Spare analog in 2
  //    uint16      linevoltage;        // 2500 bits = 25V; 1 bit = 0,01V
  //    uint16      time_stamp;

  // Output channels
  out_tueEthercatMemoryt* output_struct_ptr = (out_tueEthercatMemoryt*)(slave_->outputs);

  double max_voltage = 24.0;  // ToDo: check with Alex (has been e-mailed), Ruud and/or Arthur
  outputs_[0] = std::make_shared<MotorCommand>("MotorCommand1", (std::bitset<8>*)&output_struct_ptr->mcom1, 1);
  outputs_[1] = std::make_shared<ES030Output>("CurrentReference1", &output_struct_ptr->pwm_duty_motor_1, -6.0, 6.0);
  outputs_[2] = std::make_shared<ES030Output>("FF1", &output_struct_ptr->ff1, -max_voltage, max_voltage);
  outputs_[3] = std::make_shared<MotorCommand>("MotorCommand2", (std::bitset<8>*)&output_struct_ptr->mcom2, 2);
  outputs_[4] = std::make_shared<ES030Output>("CurrentReference2", &output_struct_ptr->pwm_duty_motor_2, -3.0, 3.0);
  outputs_[5] = std::make_shared<ES030Output>("FF2", &output_struct_ptr->ff2, -max_voltage, max_voltage);
  outputs_[6] = std::make_shared<MotorCommand>("CurrentReference3", (std::bitset<8>*)&output_struct_ptr->mcom3, 3);
  outputs_[7] = std::make_shared<ES030Output>("PWMOutput3", &output_struct_ptr->pwm_duty_motor_3, -3.0, 3.0);
  outputs_[8] = std::make_shared<ES030Output>("FF3", &output_struct_ptr->ff3, -max_voltage, max_voltage);

  digital_out_t* digital_out = &output_struct_ptr->digital_out;  // ToDo: check order
  std::bitset<8>* line_output_ptr = (std::bitset<8>*)&digital_out->line;
  outputs_[9] = std::make_shared<DigitalOutput<8> >("Enable1", line_output_ptr, 0);
  outputs_[10] = std::make_shared<DigitalOutput<8> >("Enable2", line_output_ptr, 1);
  outputs_[11] = std::make_shared<DigitalOutput<8> >("SpareDO3", line_output_ptr, 2);
  outputs_[12] = std::make_shared<DigitalOutput<8> >("SpareDO4", line_output_ptr, 3);
  //    outputs_[13] = std::make_shared<DigitalOutput<8> >("Reserved1", line_output_ptr, 4);  // Do not expose?!
  //    outputs_[14] = std::make_shared<DigitalOutput<8> >("Reserved2", line_output_ptr, 5);  // Do not expose?!
  //    outputs_[15] = std::make_shared<DigitalOutput<8> >("Reserved3", line_output_ptr, 6);  // Do not expose?!
  //    outputs_[16] = std::make_shared<DigitalOutput<8> >("Reserved4", line_output_ptr, 7);  // Do not expose?!
  outputs_[17] =
      std::make_shared<AnalogOutput<int16> >("AnalogOut1", &output_struct_ptr->analog_out_1, 12, 0.0, 10.0, -2048);
  outputs_[18] =
      std::make_shared<AnalogOutput<int16> >("AnalogOut2", &output_struct_ptr->analog_out_2, 12, 0.0, 10.0, -2048);

  //    uint8       mcom1;              // motor 1 command (0=braked, 1/3=controlled, 2=tristate)
  //    int16       pwm_duty_motor_1;   // current setpoint 1 (1 bit = 1 mA) 6A continiuous current
  //    int16       ff1;                // current feed forward 1 (1 bit = 1 mA)
  //    uint8       mcom2;              // motor 2 command (0=braked, 1/3=controlled, 2=tristate)
  //    int16       pwm_duty_motor_2;   // current setpoint 2 (1 bit = 1 mA) 3A continuous current
  //    int16       ff2;                // current feed forward 2 (1 bit = 1 mA)
  //    uint8       mcom3;              // motor 3 command (0=braked, 1/3=controlled, 2=tristate)
  //    int16       pwm_duty_motor_3;   // current setpoint 3 (1 bit = 1 mA) 3A continuous current
  //    int16       ff3;                // current feed forward 3 (1 bit = 1 mA)
  //    digital_out_t digital_out;      // digital output 8 bits
  //    int16       analog_out_1;       // analog output 1  (0V = -2048, 10V = 2047, 5V = 0 is no motion)
  //    int16       analog_out_2;       // analog output 2
  //    out_el6022t	out_el6022;	    // Data structure for RS485 communication
  //} out_tueEthercatMemoryt;
}

}  // namespace ethercat_interface
