//Prevent recursive inclusion because those are lame
#ifndef __STEPPER_INCLUDED
#define __STEPPER_INCLUDED

typedef struct Stepper_st Stepper;

//INCLUDE STATEMENTS
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

//CONSTANTS
#define NUM_STEPS 400 //The number of steps in a full 360 degree rotation
#define STEP_DELAY 0x05FFFF //The amount of time to delay between steps

//STRUCT DEFINITION
struct Stepper_st{
	int polarity;
	int position; //The current position
	uint16_t stepPin;
	GPIO_TypeDef* stepBlock;
	uint16_t dirPin;
	GPIO_TypeDef* dirBlock;
};

//FUNCTION DECLARATIONS

//Create and initialize a stepper object
//stepPin & stepBlock:  The pin and block of the step pin
//dirPin & dirBlock:    The pin and block of the direction control pin
//polarity:             -1 or +1, controls which way a positive step is.  Figure it out.
Stepper* Stepper_Initialize(GPIO_TypeDef* stepBlock, uint16_t stepPin, GPIO_TypeDef* dirBlock, uint16_t dirPin, int polarity);

//Free the memory taken up by the stepper object
void Stepper_Destroy(Stepper* stepper); 

//Declare the current position of the stepper to be position 0
//This will not actually move the stepper, just re-calibrate it
void Stepper_Calibrate(Stepper* stepper);

//Reset the stepper to its zero position
void Stepper_Reset(Stepper* stepper);

//Step a certain number of steps; a positive or negative
//number indicates direction
void Stepper_Step(Stepper* stepper, int steps);

//Move the stepper to a certain position
void Stepper_SetStep(Stepper* stepper, int step);

//Get the current step position of the stepper
//Will return a number between -199 and 200
//0 steps is considered forward
int Stepper_GetStep(Stepper* stepper);

//Set the stepper to the given angle in degrees
// 0 degrees is forward
void Stepper_SetAngle(Stepper* stepper, float angle) __attribute__((error("Function broken, don't use")));

//Get current angle of the stepper motor in degrees
float Stepper_GetAngle(Stepper* stepper) __attribute__((error("Function broken, don't use")));

#endif /*__STEPPER_INCLUDED*/