#include "servo.h"


servo::servo(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer , uint8_t controlRegisterNum)
{
	
}

void servo::setAngle(uint8_t)
{
	
}

void servo::setMaxPulseLength(double pulseLength)
{
	maxPulse = pulseLength;
}

void servo::setMinPulseLength(double pulseLength)
{
	minPulse = pulseLength;
}

void servo::setMaxAngle(uint8_t agnle)
{
	maxAngle = angle;
}

void servo::setMinAngle(uint8_t angle)
{
	minAngle = angle;
}

uint8_t getAngle(uint8_t)
{
	
}

void bankToClock(GPIO_TypeDef* bank)
{
	//initializes the correct clock that goes with the inputed bank
	if (bank == GPIOA)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	}
	else if (bank == GPIOB)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	}
	else if (bank == GPIOC)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  	}
	else if (bank == GPIOD)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	}
	else if (bank == GPIOE)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	}
	else if (bank == GPIOF)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	}
	else if (bank == GPIOG)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	}
}

void timerToClock(TIM_TypeDef* timer)
{
	  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(timer)); 
 
  if (timer == TIM1)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
  } 
  else if (timer == TIM2) 
  {     
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  }  
  else if (timer == TIM3)
  { 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  }  
  else if (timer == TIM4)
  { 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  }  
  else if (timer == TIM5)
  {      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  }  
  else if (timer == TIM6)  
  {    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  }  
  else if (timer == TIM7)
  {      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  }  
  else if (timer == TIM8)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  }  
  else if (timer == TIM9)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
   }  
  else if (timer == TIM10)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
  }  
  else if (timer == TIM11) 
  {     
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
  }  
  else if (timer == TIM12)
  {      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
  }  
  else if (timer == TIM13) 
  {       
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 
  }  
  else if(timer == TIM14)
  {     
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
  }
}
