#ifndef __SERVO_H
#define __SERVO_H

//TIM_OCInitTypeDef* controlRegister

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" 

class servo
{
	public:
		
		/*
		* Function: constructor
		* Usage: servo myServo(Pin Bank, Pin Number, Timer Number, Control Register Number);
		* Where myServo is the name of an instance of servo
		* ----------------------
		* Creates an instance of the servo class.
		*/
		
		servo(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer , uint8_t controlRegister);
	
		/*
		* Function: setAnlge
		* Usage: myServo.setAngle(angle);
		* ----------------------
		* Sets the servo attached to myServo to the specified angle
		*/
	
		void setAngle(uint8_t angle);
	
		/*
		* Function: setMaxPulseLength
		* Usage: myServo.setMaxPulseLength(Pulse Length);
		* ----------------------
		* Sets the upper end of the max pulse length to a new value
		*/
	
		void setMaxPulseLength(double pulseLength);
		
		/*
		* Function: setMinPulseLength
		* Usage: myServo.setMinPulseLength(Pulse Length);
		* ----------------------
		* Sets the lower end of the max pulse length to a new value
		*/

		void setMinPulseLength(double pulseLength);
	
		/*
		* Function: setMaxAngle
		* Usage: myServo.setMaxAngle(angle);
		* ----------------------
		* Sets the maximum angle that the servo motor can do
		*/
	
		void setMaxAngle(uint8_t angle);
	
		/*
		* Function: setMinAngle
		* Usage: myServo.setMinAngle(angle);
		* ----------------------
		* Sets the minimum angle that the servo motor can do
		*/
	
		void setMinAngle(uint8_t angle);
	
		/*
		* Function: getAngle
		* Usage: uint8_t angle = myServo.getAngle();
		* ----------------------
		* Returns the angle that the servo motor is currently set to
		*/
	
		uint8_t getAngle();
	
	
	private: 
	
		
		/* private instance variables */
	
		uint8_t currentAngle = 0; //The angle that the servo is set to
		uint8_t controlRegisterNum; //The number of the control register that is outputting pwm signal to the servo
		double maxPulse = 2.6; //The max length (in ms) the the pwm is high each period
		double minPulse = 0.8; //The min length (int ms) the pwm is high for each period
		double maxAngle = 180; //the maximum angle that a servo can be set to
		double minAngle = 0; //The minimum angle that a servo can be set to
		TIM_TypeDef* timerNum; //The number of the timer that is associated with the servo motor
		uint32_t controlRegisterOutput; //The control register (1, 2, 3, or 4) associated with the servo motor
		uint8_t gpioAltFunc;
		
		const static uint8_t frequency = 50; //The frequency of the pwm signal in Hz (20ms pwm length)
		const static uint8_t prescaler = 64; //The amount that the chosen timer is prescaled compared to the main cpu
<<<<<<< HEAD
		const static uint16_t servoPeriod = 26250 * 2; //The 
=======
		uint16_t servoPeriod;
>>>>>>> origin/master
		
		
		/* private functions */
		
		uint16_t getPinSource(uint16_t pin);
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
	
		void timerToClock(TIM_TypeDef* timer);  //sets teh correct clock for a certain timer''
	
<<<<<<< HEAD
		void setControlRegister(uint8_t registerNum, TIM_TypeDef* timer);
=======
		void setControlRegister(uint8_t, TIM_TypeDef* timer);  //initializes the control register that puts the timer into pwm mode
		
		void timerFactor(TIM_TypeDef* timer);  //gets the correct time factor that the pwm pulse length is mulitplied by
>>>>>>> origin/master
};

#endif