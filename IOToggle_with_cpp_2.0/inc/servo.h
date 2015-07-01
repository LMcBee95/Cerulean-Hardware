#ifndef __SERVO_H
#define __SERVO_H

//TIM_OCInitTypeDef* controlRegister

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" 

class servo
{
	public:
		
		servo(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer , uint8_t controlRegister);
	
		void setAngle(uint8_t angle);
	
		void setMaxPulseLength(double pulseLength);

		void setMinPulseLength(double pulseLength);
	
		void setMaxAngle(uint8_t angle);
	
		void setMinAngle(uint8_t angle);
	
		uint8_t getAngle();
	
	
	private: 
	
		
		/* private instance variables */
	
		uint8_t currentAngle = 0; //The angle that the servo is set to
		uint8_t controlRegisterNum; //The number of the control register that is oututting pwm signal to the servo
		double maxPulse = 2.6; //The max length (in ms) the the pwm is high each period
		double minPulse = 0.8; //The min length (int ms) the pwm is high for each period
		double maxAngle = 180; //the maximum angle that a servo can be set to
		double minAngle = 0; //The minimum angle that a servo can be set to
		TIM_TypeDef* timerNum;
		uint32_t controlRegisterOutput;
		uint8_t gpioAltFunc;
		
		const static uint8_t frequency = 50; //The frequency of the pwm signal in Hz (20ms pwm length)
		const static uint8_t prescaler = 64; //The amount that the chosen timer is prescaled compared to the main cpu
		uint16_t servoPeriod;
		
		
		/* private functions */
		
		uint16_t getPinSource(uint16_t pin);
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
	
		void timerToClock(TIM_TypeDef* timer);  //sets teh correct clock for a certain timer''
	
		void setControlRegister(uint8_t, TIM_TypeDef* timer);  //initializes the control register that puts the timer into pwm mode
		
		void timerFactor(TIM_TypeDef* timer);  //gets the correct time factor that the pwm pulse length is mulitplied by
};

#endif