#ifndef __EL2008_H
#define __EL2008_H

#include "ethercat_interface/slave_driver.h"

class EL2008: public SlaveDriver
{

public:

	void set_output(uint8_t output_nr, boolean do_enable);
	void toggle_output(uint8_t output_nr);
	boolean get_output(uint8_t output_nr);	
};


#endif

