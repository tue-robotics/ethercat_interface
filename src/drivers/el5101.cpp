#include "./el5101.h"

namespace ethercat_interface
{
typedef struct PACKED
{
  uint8 status;
  uint16 invalue;
  uint16 latch;
  uint32 frequency;
  uint16 period;
  uint16 window;
} in_el5101t;

EL5101::EL5101(ec_slavet* slave) : Driver(slave)
{
  in_el5101t* input_struct_ptr = (in_el5101t*)(slave_->inputs);
  uint16_t* data_ptr = &input_struct_ptr->invalue;
  inputs_[0] = std::make_shared<Encoder<uint16> >("Encoder", data_ptr);
}

}  // namespace ethercat_interface
