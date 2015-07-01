#include "servo.h"


servo::servo(GPIO_TypeDef* bank, uint16_t pin, TIM_TypeDef* timer, uint8_t controlRegister)
{
	
<<<<<<< HEAD
	timerNum = timer; //Saves the value of the timer inputted by the user
	controlRegisterNum = controlRegister; //Saves the value of the control register number inputted by the user
=======
	timerFactor(timer);  //gets the correct pulse length for the timer
	
	timerNum = timer;
	controlRegisterNum = controlRegister;
>>>>>>> origin/master
	
	
	timerToClock(timer); //Initiates the clock for the given timer
	bankToClock(bank); //Initiates the clock for a given bank

	
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Initialize the given pin for pwm
	GPIO_InitStructure.GPIO_Pin = pin;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(bank, &GPIO_InitStructure);	//initializes the structure
	
	GPIO_PinAFConfig(bank, getPinSource(pin), gpioAltFunc);
	
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / (84000000 / prescaler)) - 1;
	uint16_t PreCalPeriod = ((84000000 / prescaler) / frequency) - 1;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
	
	// Setup timebase for the given timer
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the pre scaler which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);  //initializes this part of the code
	
	setControlRegister(controlRegister, timer);
	
	// Enable the given time peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(timer, ENABLE);
	
	TIM_Cmd(timer, ENABLE); //Enable the timer given
}

void servo::timerFactor(TIM_TypeDef* timer)
{
	
	if(timer == TIM1)
	{
		servoPeriod = 26250 * 2;
	}
	else if(timer == TIM2)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM3)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM4)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM5)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM8)
	{
		servoPeriod = 26250 * 2;
	}
	else if(timer == TIM9)
	{
		servoPeriod = 26250 * 2;
	}
	else if(timer == TIM10)
	{
		servoPeriod = 26250 * 2;
	}
	else if(timer == TIM11)
	{
		servoPeriod = 26250 * 2;
	}
	else if(timer == TIM12)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM13)
	{
		servoPeriod = 26250;
	}
	else if(timer == TIM14)
	{
		servoPeriod = 26250;
	}
}

uint16_t servo::getPinSource(uint16_t pin)
{
	if(pin == GPIO_Pin_0)
	{
		return GPIO_PinSource0;
	}
	else if(pin == GPIO_Pin_1)
	{
		return GPIO_PinSource1;
	}
	else if(pin == GPIO_Pin_2)
	{
		return GPIO_PinSource2;
	}
	else if(pin == GPIO_Pin_3)
	{
		return GPIO_PinSource3;
	}
	else if(pin == GPIO_Pin_4)
	{
		return GPIO_PinSource4;
	}
	else if(pin == GPIO_Pin_5)
	{
		return GPIO_PinSource5;
	}
	else if(pin == GPIO_Pin_6)
	{
		return GPIO_PinSource6;
	}
	else if(pin == GPIO_Pin_7)
	{
		return GPIO_PinSource7;
	}
	else if(pin == GPIO_Pin_8)
	{
		return GPIO_PinSource8;
	}
	else if(pin == GPIO_Pin_9)
	{
		return GPIO_PinSource9;
	}
	else if(pin == GPIO_Pin_10)
	{
		return GPIO_PinSource10;
	}
	else if(pin == GPIO_Pin_11)
	{
		return GPIO_PinSource11;
	}
	else if(pin == GPIO_Pin_12)
	{
		return GPIO_PinSource12;
	}
	else if(pin == GPIO_Pin_13)
	{
		return GPIO_PinSource13;
	}
	else if(pin == GPIO_Pin_14)
	{
		return GPIO_PinSource14;
	}
	else if(pin == GPIO_Pin_15)
	{
		return GPIO_PinSource15;
	}
}

void servo::setAngle(uint8_t angle)
{
	//TODO: Fix it so that the CCRx changes depending upon what control register the pin is on
	
	if(controlRegisterNum == 1)
	{
		timerNum->CCR1 = (((servoPeriod + 1) / 20) * ((maxPulse - minPulse) * angle / maxAngle + minPulse));
	}
	else if(controlRegisterNum == 2)
	{
		timerNum->CCR2 = (((servoPeriod + 1) / 20) * ((maxPulse - minPulse) * angle / maxAngle + minPulse));
	}
	else if(controlRegisterNum == 3)
	{
		timerNum->CCR3 = (((servoPeriod + 1) / 20) * ((maxPulse - minPulse) * angle / maxAngle + minPulse));
	}
	else if(controlRegisterNum == 4)
	{
		timerNum->CCR4 = (((servoPeriod + 1) / 20) * ((maxPulse - minPulse) * angle / maxAngle + minPulse));
	}
<<<<<<< HEAD
	
=======
		
>>>>>>> origin/master
	currentAngle = angle;
}

void servo::setMaxPulseLength(double pulseLength)
{
	maxPulse = pulseLength;
}

void servo::setMinPulseLength(double pulseLength)
{
	minPulse = pulseLength;
}

void servo::setMaxAngle(uint8_t angle)
{
	maxAngle = angle;
}

void servo::setMinAngle(uint8_t angle)
{
	minAngle = angle;
}

uint8_t servo::getAngle()
{
	return currentAngle;
}

void servo::bankToClock(GPIO_TypeDef* bank)
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

void servo::timerToClock(TIM_TypeDef* timer)
{
 
  if (timer == TIM1)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	gpioAltFunc = GPIO_AF_TIM1;
  } 
  else if (timer == TIM2) 
  {     
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	gpioAltFunc = GPIO_AF_TIM2;
  }  
  else if (timer == TIM3)
  { 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	gpioAltFunc = GPIO_AF_TIM3;
  }  
  else if (timer == TIM4)
  { 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	gpioAltFunc = GPIO_AF_TIM4;
  }  
  else if (timer == TIM5)
  {      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	gpioAltFunc = GPIO_AF_TIM5;
  }  
  else if (timer == TIM6)  
  {   
	//Timer 6 is incapable of doing pwm
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  }  
  else if (timer == TIM7)
  {      
	//Timer 7 is incapable of doing pwm
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  }  
  else if (timer == TIM8)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	gpioAltFunc = GPIO_AF_TIM8;
  }  
  else if (timer == TIM9)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	gpioAltFunc = GPIO_AF_TIM9;
  }  
  else if (timer == TIM10)
  {      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	gpioAltFunc = GPIO_AF_TIM10;
  }  
  else if (timer == TIM11) 
  {     
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	gpioAltFunc = GPIO_AF_TIM11;
  }  
  else if (timer == TIM12)
  {      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	gpioAltFunc = GPIO_AF_TIM12;
  }  
  else if (timer == TIM13) 
  {       
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 
	gpioAltFunc = GPIO_AF_TIM13;
  }  
  else if(timer == TIM14)
  {     
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	gpioAltFunc = GPIO_AF_TIM14;
  }
}

void servo::setControlRegister(uint8_t registerNum, TIM_TypeDef* timer)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the timer to output pwm signals
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	if(registerNum == 1)
	{
		TIM_OC1Init(timer, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	else if(registerNum == 2)
	{
		TIM_OC2Init(timer, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	else if(registerNum == 3)
	{
		TIM_OC3Init(timer, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
	}
	else if(registerNum == 4)
	{
		TIM_OC4Init(timer, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}
}
