#ifndef __TEST_H
#define __TEST_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?

class led
{
	public:
	
	led();
	
	void blueOn();
	void blueOff();
	
	void redOn();
	void redOff();
	
	void greenOn();
	void greenOff();
	
	void orangeOn();
	void oragneOff();
	
	private:
		
	
	
};

#endif