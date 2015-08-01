#ifndef __PWM_H
#define __PWM_H

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
 *     |	A1	TIM2, TIM5	|	B1	TIM3	|	C7	TIM3, TIM8	|	E6	TIM9	|	H9	TIM12	|
 *	|	A2	TIM2, TIM5	|	B3	TIM2	|	C8	TIM3, TIM8	|	E9	TIM1	|	H10	TIM5	|
 *	|	A3	TIM2, TIM5	|	B4	TIM3	|	C9	TIM3, TIM8	|	E10	TIM1	|	H11	TIM5	|
 *	|	A5	TIM2		|	B5 	TIM3	|					|	E13	TIM1	|	H12 TIM5	|
 *	|	A6	TIM3		|	B6	TIM4	|	D12	TIM4		|	E14	TIM1	|				|
 *	|	A7	TIM3		|	B7 	TIM4	|	D13	TIM4		|				|	I0	TIM5	|
 *	|	A8	TIM1		|	B8	TIM4	|	D14	TIM4		|	F6	TIM10	|	I2	TIM8	|
 *	|	A9	TIM1		|	B9	TIM4	|	D15	TIM4		|	F7	TIM11	|	I5	TIM8	|
 *	|	A10 TIM1			|	B10	TIM2	|					|	F8	TIM13	|	I6	TIM8	|
 *	|	A11	TIM1		|	B11 TIM2		|					|	F9	TIM14	|	I7	TIM8	|
 *	|	A15	TIM2		|	B14	TIM12	|					|				|				|
 *	|					|	B15 TIM12	|					|				|				|   
 * ------------------------------------------------------------------------------------------
 ********************************************************************************************/
 
 
 class pwm
 {
	 public:
		 
		/*
		* Function: constructor
		* Usage: owm myPwm(GPIOA , GPIO_Pin_0, TIM2, 1);
		* Where myPwm is the name of an instance of servo
		* ----------------------
		* Creates an instance of the servo class.
		*/
		
		pwm(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer , uint8_t controlRegister, uint32_t inputFrequency = 100000);
		
	 
		void set(uint8_t dutyCycle);
		 
	 private:

		/* private instance variables */
	 
		uint8_t controlRegisterNum; //The number of the control register that is outputting pwm signal to the servo
		TIM_TypeDef* timerNum; //The number of the timer that is associated with the servo motor
		uint32_t controlRegisterOutput; //The control register (1, 2, 3, or 4) associated with the servo motor
		uint8_t gpioAltFunc;
		uint32_t period;  //the period of the pwm in clock ticks
		
		/* private functions */
		
		
		uint32_t frequency;
	 
		uint16_t getPinSource(uint16_t pin);
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
	
		void timerToClock(TIM_TypeDef* timer);  //sets teh correct clock for a certain timer''
	
		void setControlRegister(uint8_t controlRegister, TIM_TypeDef* timer);  //initializes the control register that puts the timer into pwm mode
		
		void timerFactor(TIM_TypeDef* timer);  //gets the correct time factor that the pwm pulse length is mulitplied by
 };
 
 
 
 
 #endif