#include "gpio.h"


gpio::gpio(GPIO_TypeDef* inputedBank, uint16_t pinNum)
{	
	pin = pinNum;  //assigns the pinNumber to an instance variable to be used in the other class functions
	bank = inputedBank;  //assings the inputed bank to the instance variable that stores the bank
	
	/* Struct used in the initialization of the pin */
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	
	//enables the correct clock
	bankToClock(inputedBank);

	GPIO_InitStructure.GPIO_Pin = pinNum;  //assigns the pinNum
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(inputedBank, &GPIO_InitStructure);

}


void gpio::on()
{
	GPIO_SetBits(bank, pin); 
}

void gpio::off()
{
	GPIO_ResetBits(bank, pin); 
}

void gpio::toggle()
{
	GPIO_ToggleBits(bank, pin); 
}

void gpio::bankToClock(GPIO_TypeDef* bank)
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