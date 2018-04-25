#pragma once

#include <queue>

#include "ethercat_interface/driver.h"

namespace ethercat_interface
{
// EL6022
#define CHANNEL_1 0
//#define CHANNEL_2               1
#define CHANNEL_NUM 1
#define MAX_TRIALS 30
#define MAX_OUT_QUEUE_SIZE 220
#define RS485_MAX_DATA_LENGTH 22

// CONTROL MASKS
#define TRANSMIT_REQUEST 0x01
#define RECEIVE_ACCEPTED 0x02
#define INIT_REQUEST 0x04
#define SEND_CONTINUOUS 0x08

// STATUS MASKS
#define TRANSMIT_ACCEPTED 0x01
#define RECEIVE_REQUEST 0x02
#define INIT_ACCEPTED 0x04
#define BUFFER_FULL 0x08
#define PARITY_ERROR 0x10
#define FRAMING_ERROR 0x20
#define OVERRUN_ERROR 0x40

typedef enum RS485_BAUDRATE {
  RS485_300_BAUD = 1,
  RS485_600_BAUD,
  RS485_1200_BAUD,
  RS485_2400_BAUD,
  RS485_4800_BAUD,
  RS485_9600_BAUD,
  RS485_19200_BAUD,
  RS485_38400_BAUD,
  RS485_57600_BAUD,
  RS485_115200_BAUD
} RS485_BAUDRATE;

typedef enum RS485_DATA_FRAME {
  RS485_7B_EP_1S = 1,
  RS485_7B_OP_1S,
  RS485_8B_NP_1S,
  RS485_8B_EP_1S,
  RS485_8B_OP_1S,
  RS485_7B_EP_2S,
  RS485_7B_OP_2S,
  RS485_8B_NP_2S,
  RS485_8B_EP_2S,
  RS485_8B_OP_2S
} RS485_DATA_FRAME;

typedef enum RS485_HANDSHAKE { XON_XOFF_DISABLE = 0, XON_XOFF_ENABLE } RS485_HANDSHAKE;

typedef enum RS485_DUPLEX { RS485_FULL_DUPLEX = 0, RS485_HALF_DUPLEX } RS485_DUPLEX;

typedef enum state_el6022t { START, INIT_REQ, INIT_WAIT, PREP_REQ, PREP_WAIT, RUN } state_el6022t;

typedef struct PACKED
{
  uint8 control;
  uint8 output_length;
  uint8 buffer_out[RS485_MAX_DATA_LENGTH];
} out_el6022t;

typedef struct PACKED
{
  uint8 status;
  uint8 input_length;
  uint8 buffer_in[RS485_MAX_DATA_LENGTH];
} in_el6022t;

// WARNING, the bits are numbered in reversed order
typedef union PACKED
{
  struct PACKED
  {
    uint8 spare_di_1 : 1;  // bit 0
    uint8 spare_di_2 : 1;
    uint8 spare_di_3 : 1;
    uint8 spare_di_4 : 1;
    uint8 reserved_1 : 1;
    uint8 reserved_2 : 1;
    uint8 reserved_3 : 1;
    uint8 power_status : 1;  // bit 7
  } line;
  uint8 port;
} digital_in_t;

/*
typedef struct PACKED {
        digital_in_t digital_in;        // Digital inputs
        uint16 encoder_1;               // Encoder 1
        uint16 encoder_2;               // Encoder 2
        uint16 encoder_3;               // Encoder 3
        uint16 current_1;               // Current 1
        uint16 current_2;               // Current 2
        uint16 current_3;               // Current 3
        uint16 calipher_1;              // calipher 1 (1 bit = 0.01 mm)
        uint16 calipher_2;              // calipher 2 (1 bit = 0.01 mm)
        uint16 force_1;                 // Analog ADC value of force sensor input 1
        uint16 force_2;                 // Analog ADC value of force sensor input 2
        uint16 force_3;                 // Analog ADC value of force sensor input 3
        uint16 position_1;              // Analog ADC value of position sensor 1
        uint16 position_2;              // Analog ADC value of position sensor 2
        uint16 position_3;              // Analog ADC value of position sensor 3
        uint16 spare_ai_1;              // Spare analog in 1
        uint16 spare_ai_2;              // Spare analog in 2
        uint16 time_stamp;              // Time stamp (1 bit equals 256 ns)
    } in_tueEthercatMemoryt; */

typedef struct PACKED
{
  uint8 mstate1;            // motor state 1
  uint32 encoder_1;         // Encoder 1
  uint32 timestamp1;        // Time stamp encoder 1 (Only changes, if encoder changes)
  int16 velocity1;          // Velocity encoder 1; 1 bit=0.1 rad/s; depending on parameters on slave
  int16 current_1;          // current on PWM 1 (1 bit = 1 mA)
  uint8 mstate2;            // motor state 2
  uint32 encoder_2;         // Encoder 2
  uint32 timestamp2;        // Time stamp encoder 2 (Only changes, if encoder changes)
  int16 velocity2;          // Velocity encoder 2; 1 bit=0.1 rad/s; depending on parameters on slave
  int16 current_2;          // current on PWM 2 (1 bit = 1 mA)
  uint8 mstate3;            // motor state 3
  uint32 encoder_3;         // Encoder 3
  uint32 timestamp3;        // Time stamp encoder 3 (Only changes, if encoder changes)
  int16 velocity3;          // Velocity encoder 3; 1 bit=0.1 rad/s; depending on parameters on slave
  int16 current_3;          // current on PWM 2 (1 bit = 1 mA)
  digital_in_t digital_in;  // digital input 8 bits
  uint16 calipher_1;        // calipher 1 (1 bit = 0.01 mm)
  uint16 calipher_2;        // calipher 2 (1 bit = 0.01 mm)
  uint16 force_1;           // Analog ADC value of force sensor input 1
  uint16 force_2;           // Analog ADC value of force sensor input 2
  uint16 force_3;           // Analog ADC value of force sensor input 3
  uint16 position_1;        // Analog ADC value of position sensor 1
  uint16 position_2;        // Analog ADC value of position sensor 2
  uint16 position_3;        // Analog ADC value of position sensor 3
  uint16 spare_ai_1;        // Spare analog in 1
  uint16 spare_ai_2;        // Spare analog in 2
  uint16 linevoltage;       // 2500 bits = 25V; 1 bit = 0,01V
  uint16 time_stamp;
  in_el6022t in_el6022;  // Data structure for RS485 communication
} in_tueEthercatMemoryt;

// WARNING, the bits are numbered in reversed order
typedef union PACKED
{
  struct PACKED
  {
    uint8 enable_1 : 1;  // bit 0
    uint8 enable_2 : 1;
    uint8 spare_do_3 : 1;
    uint8 spare_do_4 : 1;
    uint8 reserved_1 : 1;
    uint8 reserved_2 : 1;
    uint8 reserved_3 : 1;
    uint8 reserved_4 : 1;  // bit 7
  } line;
  uint8 port;
} digital_out_t;

/*
typedef struct PACKED {
    digital_out_t digital_out;      // Digital outputs
    int16 pwm_duty_motor_1;         // PWM duty cycle for motor 1 (limited from -1000 up to 1000, 0 is no motion)
    int16 pwm_duty_motor_2;         // PWM duty cycle for motor 2
    int16 pwm_duty_motor_3;         // PWM duty cycle for motor 3
    int16 analog_out_1;             // Analog output 1  (0V = -2048, 10V = 2047, 5V = 0 is no motion)
    int16 analog_out_2;             // Analog output 2
} out_tueEthercatMemoryt;*/

typedef struct PACKED
{
  uint8 mcom1;                // motor 1 command (0=braked, 1/3=controlled, 2=tristate)
  int16 pwm_duty_motor_1;     // current setpoint 1 (1 bit = 1 mA) 6A continiuous current
  int16 ff1;                  // current feed forward 1 (1 bit = 1 mA)
  uint8 mcom2;                // motor 2 command (0=braked, 1/3=controlled, 2=tristate)
  int16 pwm_duty_motor_2;     // current setpoint 2 (1 bit = 1 mA) 3A continuous current
  int16 ff2;                  // current feed forward 2 (1 bit = 1 mA)
  uint8 mcom3;                // motor 3 command (0=braked, 1/3=controlled, 2=tristate)
  int16 pwm_duty_motor_3;     // current setpoint 3 (1 bit = 1 mA) 3A continuous current
  int16 ff3;                  // current feed forward 3 (1 bit = 1 mA)
  digital_out_t digital_out;  // digital output 8 bits
  int16 analog_out_1;         // analog output 1  (0V = -2048, 10V = 2047, 5V = 0 is no motion)
  int16 analog_out_2;         // analog output 2
  out_el6022t out_el6022;     // Data structure for RS485 communication
} out_tueEthercatMemoryt;

class TUeES030 : public Driver
{
public:
  TUeES030(ec_slavet* slave);

private:
};  // End of class TUeES030

}  // namespace ethercat_interface
