#include "ethercat_interface/ethercat_includes.h"
#include "./tuees030.h"
#include "../io_interfaces/analog_input.h"
#include "../io_interfaces/analog_output.h"
#include "../io_interfaces/encoder.h"

TUeES030::TUeES030(std::string name, ec_slavet *slave) : EtherCatDriver(name, slave)
{
    // Input channels
    in_tueEthercatMemoryt* input_struct_ptr = (in_tueEthercatMemoryt*)(ec_slave_->inputs);
    // ToDo: mstate1 [0]
    inputs_[1] = std::make_shared<Encoder<uint32> >("Encoder1", &input_struct_ptr->encoder_1);
    inputs_[2] = std::make_shared<AnalogInput<int16> >("Current1", &input_struct_ptr->current_1);
    // ToDo: mstate2 [3]
    inputs_[4] = std::make_shared<Encoder<uint32> >("Encoder2", &input_struct_ptr->encoder_2);
    inputs_[5] = std::make_shared<AnalogInput<int16> >("Current2", &input_struct_ptr->current_2);
    // ToDo: mstate3 [6]
    inputs_[7] = std::make_shared<Encoder<uint32> >("Encoder3", &input_struct_ptr->encoder_3);
    inputs_[8] = std::make_shared<AnalogInput<int16> >("Current3", &input_struct_ptr->current_3);
    //    digital_in_t digital_in;                    // digital input 8 bits [10, 11, 12, 13, 14, 15, 16, 17]
    //    uint16      calipher_1;			// calipher 1 (1 bit = 0.01 mm) [18]
    //    uint16      calipher_2;			// calipher 2 (1 bit = 0.01 mm) [19]
    inputs_[20] = std::make_shared<AnalogInput<uint16> >("Force1", &input_struct_ptr->force_1);
    inputs_[21] = std::make_shared<AnalogInput<uint16> >("Force2", &input_struct_ptr->force_2);
    inputs_[22] = std::make_shared<AnalogInput<uint16> >("Force3", &input_struct_ptr->force_3);
    inputs_[23] = std::make_shared<AnalogInput<uint16> >("Pos1", &input_struct_ptr->position_1);
    inputs_[24] = std::make_shared<AnalogInput<uint16> >("Pos2", &input_struct_ptr->position_2);
    inputs_[25] = std::make_shared<AnalogInput<uint16> >("Pos3", &input_struct_ptr->position_3);
    inputs_[26] = std::make_shared<AnalogInput<uint16> >("SpareAI1", &input_struct_ptr->spare_ai_1);
    inputs_[27] = std::make_shared<AnalogInput<uint16> >("SpareAI2", &input_struct_ptr->spare_ai_2);
    inputs_[28] = std::make_shared<AnalogInput<uint16> >("Linevoltage", &input_struct_ptr->linevoltage);

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
    out_tueEthercatMemoryt* output_struct_ptr = (out_tueEthercatMemoryt*)(ec_slave_->outputs);
    // ToDo: mcom1 [0]
    // ToDo: pwm_duty_motor_1 [1]
    // ToDo: ff1; [2]
    // ToDo: mcom2 [3]
    // ToDo: pwm_duty_motor_2 [4]
    // ToDo: ff2; [4]
    // ToDo: mcom3 [5]
    // ToDo: pwm_duty_motor_3 [6]
    // ToDo: ff3; [7]
    // ToDo: digital out [8, 9, 10, 11, 12, 13, 14, 15]
    outputs_[16] = std::make_shared<AnalogOutput<int16> >("AnalogOut1", &output_struct_ptr->analog_out_1, 12, 0.0, 10.0, -2048);
    outputs_[17] = std::make_shared<AnalogOutput<int16> >("AnalogOut2", &output_struct_ptr->analog_out_2, 12, 0.0, 10.0, -2048);

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