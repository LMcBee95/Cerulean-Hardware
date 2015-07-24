#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" 

/******************************************************************************************** 
 *	Pin and Timer Combinations for PWM
 * 	note 1: The pins attached to timer 1 or 8 are not able to 
 *  	  be set up using the code below.
 *	note 2: Timers 6 and 7 are unable to be used for PWM.
 *
 *	-----------------------------------------------------------------------------------------
 *	|	A0  TIM2, TIM5	|	B0	TIM3	|	C6	TIM3, TIM8	|	E5	TIM9	|	H6	TIM12	|
 *  |	A1	TIM2, TIM5	|	B1	TIM3	|	C7	TIM3, TIM8	|	E6	TIM9	|	H9	TIM12	|
 *	|	A2	TIM2, TIM5	|	B3	TIM2	|	C8	TIM3, TIM8	|	E9	TIM1	|	H10	TIM5	|
 *	|	A3	TIM2, TIM5	|	B4	TIM3	|	C9	TIM3, TIM8	|	E10	TIM1	|	H11	TIM5	|
 *	|	A5	TIM2		|	B5 	TIM3	|					|	E13	TIM1	|	H12 TIM5	|
 *	|	A6	TIM3		|	B6	TIM4	|	D12	TIM4		|	E14	TIM1	|				|
 *	|	A7	TIM3		|	B7 	TIM4	|	D13	TIM4		|				|	I0	TIM5	|
 *	|	A8	TIM1		|	B8	TIM4	|	D14	TIM4		|	F6	TIM10	|	I2	TIM8	|
 *	|	A9	TIM1		|	B9	TIM4	|	D15	TIM4		|	F7	TIM11	|	I5	TIM8	|
 *	|	A10 TIM1		|	B10	TIM2	|					|	F8	TIM13	|	I6	TIM8	|
 *	|	A11	TIM1		|	B11 TIM2	|					|	F9	TIM14	|	I7	TIM8	|
 *	|	A15	TIM2		|	B14	TIM12	|					|				|				|
 *	|					|	B15 TIM12	|					|				|				|   
 * ------------------------------------------------------------------------------------------
 ********************************************************************************************/
class servo
{
	public:
		
		/*
		* Function: constructor
		* Usage: servo myServo(GPIOA , GPIO_Pin_0, TIM2, 1);
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
		uint16_t servoPeriod;
	
		/* private functions */
		
		uint16_t getPinSource(uint16_t pin);
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
	
		void timerToClock(TIM_TypeDef* timer);  //sets the correct clock for a certain timer''
	
		void setControlRegister(uint8_t controlRegister, TIM_TypeDef* timer);  //initializes the control register that puts the timer into pwm mode
		
		void timerFactor(TIM_TypeDef* timer);  //gets the correct time factor that the pwm pulse length is mulitplied by
};

#endif