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

//Use byte to control horizontal and vertical steppers
uint32_t Stepper_UseByte(uint8_t byte, Stepper* horizontal, Stepper* vertical)
{
  //Declare return int
  uint32_t position;
  uint16_t horzPos;
  uint16_t vertPos;

  //Split up that byte
  uint8_t horzDir = byte>>7;          //Horizontal direction
  uint8_t vertDir = (byte>>3)&0x01;   //Vertical direction
  uint8_t horzSteps = (byte>>4)&0x07; //Horizontal steps
  uint8_t vertSteps = byte & 0x07;    //Vertical steps
  
  //Convert steps to be positive or negative based on direction
  horzSteps *= (-1 + 2*horzDir);  //0 is negative 1 is positive
  vertSteps *= (-1 + 2*vertDir);
  
  //Move stepper motors
  Stepper_Step(horizontal, horzSteps);
  Stepper_Step(vertical, vertSteps);
  
  //Return position of stepper motors
  horzPos = Stepper_GetStep(horizontal);
  vertPos = Stepper_GetStep(vertical);
  position = horzPos<<16 + vertPos;
  return position;  
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
	//Stepper_SetStep(stepper, 0);
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
	
	steps = steps % NUM_STEPS; //Remove redundant rotation
	steps += steps < 0 ? NUM_STEPS : 0; //Ensure is not negative
	steps = steps>NUM_STEPS/2 ? NUM_STEPS/2-steps : steps;
	//steps = Normalize(steps); //Only rotate 180 degrees or less to get there
	Stepper_Step(stepper, steps);
}

//Get the current step of the stepper
int Stepper_GetStep(Stepper* stepper)
{
	return stepper -> position;
}

//Set the stepper to the given angle in tenths of a degree
void Stepper_SetAngle(Stepper* stepper, int angle)
{
	int step = NUM_STEPS * angle / 3600;
	//Stepper_SetStep(stepper, step);
}

//Get current angle of the stepper motor in tenths of a degree
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
	steps += steps < 0 ? NUM_STEPS : 0; //Ensure is not negative
	if(steps > NUM_STEPS/2)    //Turn > 180 to a negative angle
	{
		steps = NUM_STEPS/2 - steps;
	}
	return steps;
}