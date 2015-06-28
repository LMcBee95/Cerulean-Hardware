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
	
	stepper -> ticksSinceLastChange = 0;
	stepper -> stepPinPolarity = 0;
	stepper -> stepBuffer = 0;
	
	Stepper_Disable(stepper);
	
	return stepper;
}

void Stepper_Calibrate(Stepper* stepper)
{
	stepper -> position = 0;
}

void Stepper_Destroy(Stepper* stepper)
{
	Stepper_Disable(stepper);
	free(stepper);
}

void Stepper_Disable(Stepper* stepper)
{
	if(STEPPER_ENABLE_INVERTED)
		GPIO_SetBits(stepper->enableBlock, stepper->enablePin);
	else
		GPIO_ResetBits(stepper->enableBlock, stepper->enablePin);
}

void Stepper_DoubleStep(Stepper* stepper1, Stepper* stepper2, int steps)
{
	int i; //For iteration
	
	for(i=0; i<steps; i++) //Assumes that the steppers use the same block
	{
		GPIO_SetBits(stepper1->stepBlock, stepper1->stepPin | stepper2->stepPin);
		Delay(STEP_DELAY);
		GPIO_ResetBits(stepper1->stepBlock, stepper1->stepPin | stepper2->stepPin);
		Delay(STEP_DELAY);
	}
}

void Stepper_Enable(Stepper* stepper)
{
	if(STEPPER_ENABLE_INVERTED)
		GPIO_ResetBits(stepper->enableBlock, stepper->enablePin);
	else
		GPIO_SetBits(stepper->enableBlock, stepper->enablePin);
}

int Stepper_GetStep(Stepper* stepper)
{
	return stepper -> position;
}

void Stepper_Reset(Stepper* stepper)
{
	//Stepper_SetStep(stepper, 0);
}

int Stepper_SetDirection(Stepper* stepper, int steps)
{
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
	return steps;
}

void Stepper_SetStep(Stepper* stepper, int step)
{
	int steps = step - stepper -> position; //Determine number of steps to take
	
	steps = steps % NUM_STEPS; //Remove redundant rotation
	steps += steps < 0 ? NUM_STEPS : 0; //Ensure is not negative
	steps = steps>NUM_STEPS/2 ? NUM_STEPS/2-steps : steps;
	//steps = Normalize(steps); //Only rotate 180 degrees or less to get there
	Stepper_Step(stepper, steps);
}

void Stepper_Step(Stepper* stepper, int steps)
{
	Stepper_Enable(stepper);
	stepper->stepBuffer += steps;
	
	Stepper_SetDirection(stepper, stepper->stepBuffer);
}

void Stepper_StepTogether(Stepper* stepper1, Stepper* stepper2, int steps1, int steps2)
{
  int min;
  
  int absSteps1;
  int absSteps2; //absolute values of the number of steps
  
  Stepper_Enable(stepper1);
  Stepper_Enable(stepper2);
  
  absSteps1 = Stepper_SetDirection(stepper1, steps1);
  absSteps2 = Stepper_SetDirection(stepper2, steps2);
  
  steps1 = steps1<0? steps1 + min : steps1 - min;
  steps2 = steps2<0? steps2 + min : steps2 - min;
  
  min = absSteps1<absSteps2 ? absSteps1 : absSteps2; //Find the minimum number of steps
  Stepper_DoubleStep(stepper1, stepper2, min);
  if(absSteps1-min>0){Stepper_Step(stepper1, steps1);}
  if(absSteps2-min>0){Stepper_Step(stepper2, steps2);}
  
}

uint8_t Stepper_UseByte(uint8_t byte, Stepper* horizontal, Stepper* vertical)
{
  //Declare return int
  uint32_t position;
  uint16_t horzPos;
  uint16_t vertPos;

  //Split up that byte
  uint8_t horzDir = byte>>7&0x01;          //Horizontal direction
  uint8_t vertDir = (byte>>3)&0x01;   //Vertical direction
  uint8_t horzSteps = (byte>>4)&0x07; //Horizontal steps
  uint8_t vertSteps = byte & 0x07;    //Vertical steps
  
  //Convert steps to be positive or negative based on direction 
  int correctedHorzSteps = horzDir==1 ? horzSteps : -horzSteps;
  int correctedVertSteps = vertDir==1 ? vertSteps : -vertSteps;
  
  //Move stepper motors
  //Stepper_StepTogether(horizontal, vertical, correctedHorzSteps, correctedVertSteps);
  Stepper_Step(horizontal, correctedHorzSteps);
  Stepper_Step(vertical, correctedVertSteps);
  
  //Return position of stepper motors
  
 return Stepper_GetStep(horizontal) & 0xFF;
  vertPos = Stepper_GetStep(vertical);
  position = horzPos<<16 + vertPos;
  
  
  //return position;  
}

void Stepper_Update(Stepper* stepper)
{
	if(stepper->ticksSinceLastChange >= TICKS_PER_STEP)
	{
		stepper->ticksSinceLastChange = 0;
		if(stepper->stepBuffer != 0)
		{
			if(stepper->stepPinPolarity == 1)
			{
				stepper->stepPinPolarity = 0;
				
				//brings the step buffer closer to 0, whether it is positive or negative
				if (stepper->stepBuffer < 0)
					stepper->stepBuffer++;
				else
					stepper->stepBuffer--;
				
				GPIO_ResetBits(stepper->stepBlock,stepper->stepPin);
			}
			else
			{
				stepper->stepPinPolarity = 1;
				GPIO_SetBits(stepper->stepBlock,stepper->stepPin);
			}
		}
	}
	stepper->ticksSinceLastChange++;
}

//Define static functions

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