#include "./tuees030.h"
#include "analog_input.h"
#include "encoder.h"

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


}
