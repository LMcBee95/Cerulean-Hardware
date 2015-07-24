#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

class gpio
{
	public:
		
		/*
		* Function: constructor
		* Usage: gpio pin(Pin Bank, Pin Number);
		* Where pin is the name of an instance of gpio
		* ----------------------
		* Creates an instance of the gpio class.
		*/

		gpio(GPIO_TypeDef* bank, uint16_t pinNum);
		
		/*
		* Function: on
		* Usage: pin.on();
		* ----------------------
		* Turns on the pin associated with that class.
		*/
		
		void on();
		
		/*
		* Function: off
		* Usage: pin.off();
		* ----------------------
		* Turns off the pin associated with that class.
		*/
		
		void off();
		
		/*
		* Function: toggle
		* Usage: pin.toggle();
		* ----------------------
		* Changes the state of the pin to the opposite state that it is currently in.
		*/
		
		void toggle();
	
	private:
		
		/* private instance variables */
		
		uint16_t pin;
		GPIO_TypeDef* bank;
		
		/* private functions */
		
		void bankToClock(GPIO_TypeDef* bank); //gets the correct clock bassed on the inputed bank
	
};


#endif