//Prevent recursive inclusion because those are lame
#ifndef __STEPPER_INCLUDED
#define __STEPPER_INCLUDED

typedef struct Stepper_st Stepper;

//INCLUDE STATEMENTS
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <math.h>
#include <stdlib.h>

//CONSTANTS
#define NUM_STEPS 400       //The number of steps in a full 360 degree rotation
#define STEP_DELAY 0x01FFFF //The amount of time to delay between steps
#define STEPPER_ENABLE_INVERTED 1   //Whether or not the enable pin is inverted

//STRUCT DEFINITION
struct Stepper_st{
	int polarity; //The polarity of the stepper
	int position; //The current position
	int angle;
	GPIO_TypeDef* stepBlock;
	uint16_t stepPin;
	
	GPIO_TypeDef* dirBlock;
	uint16_t dirPin;
	
	GPIO_TypeDef* enableBlock;
	uint16_t enablePin;
};

//FUNCTION DECLARATIONS

//Create and initialize a stepper object
//stepPin & stepBlock:  The pin and block of the step pin
//dirPin & dirBlock:    The pin and block of the direction control pin
//polarity:             -1 or +1, controls which way a positive step is.  Figure it out.
Stepper* Stepper_Initialize(
	GPIO_TypeDef* stepBlock, uint16_t stepPin,
	GPIO_TypeDef* dirBlock, uint16_t dirPin,
	GPIO_TypeDef* enableBlock, uint16_t enablePin,
	int polarity);

//Free the memory taken up by the stepper object
void Stepper_Destroy(Stepper* stepper); 

//Declare the current position of the stepper to be position 0
//This will not actually move the stepper, just re-calibrate it
void Stepper_Calibrate(Stepper* stepper);

//Turn on the stepper and allow it to hold its position
void Stepper_Enable(Stepper* stepper);

//Disable the stepper so it stops holding its position
void Stepper_Disable(Stepper* stepper);

//Reset the stepper to its zero position
void Stepper_Reset(Stepper* stepper)
__attribute((warning("Function has not been tested thoroughly, results not guaranteed.")));

//Step a certain number of steps; a positive or negative
//number indicates direction
void Stepper_Step(Stepper* stepper, int steps);

//Move the stepper to a certain position
void Stepper_SetStep(Stepper* stepper, int step)
__attribute((warning("Function has not been tested thoroughly, results not guaranteed.")));

//Get the current step position of the stepper
//Will return a number between -199 and 200
//0 steps is considered forward
int Stepper_GetStep(Stepper* stepper);

//Set the stepper to the given angle in tenths of a degree
// 0 degrees is forward
void Stepper_SetAngle(Stepper* stepper, int angle)
__attribute((warning("Function has not been tested thoroughly, results not guaranteed.")));

//Get current angle of the stepper motor in tenths of a degree
int Stepper_GetAngle(Stepper* stepper);

#endif /*__STEPPER_INCLUDED*/