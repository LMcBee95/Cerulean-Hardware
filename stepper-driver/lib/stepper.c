#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "stepper.h"


//STATIC FUNCTION DECLARATIONS

//Delay a given amount of clock cycles
static void Delay(__IO uint32_t nCount);

//Turn a number of steps into the range -199 to 200 to avoid extra unnecessary rotation
static int Normalize(int steps);

//FUNCTION DEFINITIONS
Stepper* Stepper_Initialize(
	GPIO_TypeDef* stepBlock, uint16_t stepPin,
	GPIO_TypeDef* dirBlock, uint16_t dirPin,
	GPIO_TypeDef* enableBlock, uint16_t enablePin,
	int polarity)
{
	Stepper* stepper = malloc(sizeof(Stepper));
	stepper -> stepPin = stepPin;
	stepper -> stepBlock = stepBlock;
	
	stepper -> dirPin = dirPin;
	stepper -> dirBlock = dirBlock;
	
	stepper -> enablePin = enablePin;
	stepper -> enableBlock = enableBlock;
	
	stepper -> polarity = polarity;
	stepper -> position = 0;
	
	Stepper_Disable(stepper);
	
	return stepper;
}

//Step a certain number of steps; can be positive or negative
void Stepper_Step(Stepper* stepper, int steps)
{
	int i; //For iteration
	Stepper_Enable(stepper);
	
	steps *= stepper -> polarity;
	stepper -> position += steps;
	stepper -> position = Normalize(stepper -> position);
	
	if (steps>0) //Positive direction
	{
		GPIO_SetBits(stepper->dirBlock, stepper->dirPin);
	}
	else //Negative direction
	{
		GPIO_ResetBits(stepper->dirBlock, stepper->dirPin);
		steps *= -1;
	}
	
	for(i=0; i<steps; i++)
	{
		GPIO_SetBits(stepper->stepBlock, stepper->stepPin);
		Delay(STEP_DELAY);
		GPIO_ResetBits(stepper->stepBlock, stepper->stepPin);
		Delay(STEP_DELAY);
	}
}

//Turn on the stepper and allow it to hold its position
void Stepper_Enable(Stepper* stepper)
{
	if(STEPPER_ENABLE_INVERTED)
		GPIO_ResetBits(stepper->enableBlock, stepper->enablePin);
	else
		GPIO_SetBits(stepper->enableBlock, stepper->enablePin);
}

//Disable the stepper so it stops holding its position
void Stepper_Disable(Stepper* stepper)
{
	if(STEPPER_ENABLE_INVERTED)
		GPIO_SetBits(stepper->enableBlock, stepper->enablePin);
	else
		GPIO_ResetBits(stepper->enableBlock, stepper->enablePin);
}

void Stepper_Calibrate(Stepper* stepper)
{
	stepper -> position = 0;
}

void Stepper_Reset(Stepper* stepper)
{
	Stepper_SetStep(stepper, 0);
}

void Stepper_Destroy(Stepper* stepper)
{
	Stepper_Disable(stepper);
	free(stepper);
}

//Move the stepper to a certain step
void Stepper_SetStep(Stepper* stepper, int step)
{
	int steps = step - stepper -> position; //Determine number of steps to take
	
	steps = Normalize(steps); //Only rotate 180 degrees or less to get there
	Stepper_Step(stepper, steps);
}

//Get the current step of the stepper
int Stepper_GetStep(Stepper* stepper)
{
	return stepper -> position;
}

//Set the stepper to the given angle in degrees
void Stepper_SetAngle(Stepper* stepper, int angle)
{
	int step = NUM_STEPS * angle / 3600;
	Stepper_SetStep(stepper, step);
}

//Get current angle of the stepper motor in degrees
int Stepper_GetAngle(Stepper* stepper)
{
	return ((stepper -> position)  * 3600 /  NUM_STEPS);
}


static void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

static int Normalize(int steps)
{
	steps = steps % NUM_STEPS; //Cut out extra 360 degree rotations
	if(steps > NUM_STEPS/2)    //Turn > 180 to a negative angle
	{
		steps = NUM_STEPS/2 - steps;
	}
	return steps;
}