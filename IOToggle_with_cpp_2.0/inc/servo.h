#ifndef __SERVO_H
#define __SERVO_H

//TIM_OCInitTypeDef* controlRegister

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" 

class servo
{
	public:
		
		servo(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer , uint8_t controlRegisterNum);
	
		void setAngle(uint8_t angle);
	
		void setMaxPulseLength(double pulseLength);

		void setMinPulseLength(double pulseLength);
	
		void setMaxAngle(uint8_t angle);
	
		void setMinAngle(uint8_t angle);
	
		uint8_t getAngle();
	
	
	private: 
	
		
		/* private instance variables */
	
		uint8_t angle = 0; //The angle that the servo is set to
		uint8_t controlRegisterNum; //The number of the control register that is oututting pwm signal to the servo
		double maxPulse; //The max length (in ms) the the pwm is high each period
		double minPulse; //The min length (int ms) the pwm is high for each period
		double maxAngle; //the maximum angle that a servo can be set to
		double minAngle; //The minimum angle that a servo can be set to
	
		
		/* private functions */
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
	
		void timerToClock(TIM_TypeDef* timer);  //sets teh correct clock for a certain timer''
	
		void setControlRegister(uint8_t);
};

#endif