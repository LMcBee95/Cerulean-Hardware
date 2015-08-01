#include "serial.h"

serial::serial(GPIO_TypeDef* bankPinTx, uint16_t pinTx, GPIO_TypeDef* bankPinRx, uint16_t pinRx, USART_TypeDef* USARTx, uint32_t baudrate, uint8_t interruptFlag)
{
	thisUsart = USARTx;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	usartToClock(USARTx); //Initialize the clock for the given USART
	
	bankToClock(bankPinRx); //Initiates the clock for the bank of the Rx pin
	bankToClock(bankPinTx); //Initiates the clock for the bank of the Tx pin
	
	//Initialize the Rx pin
	GPIO_InitStructure.GPIO_Pin =  pinRx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(bankPinRx, &GPIO_InitStructure);
	
	//Initialize the Tx pin
	GPIO_InitStructure.GPIO_Pin =  pinTx;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(bankPinTx, &GPIO_InitStructure);
	
	uint16_t pinSourceRx = getPinSource(pinRx);
	uint16_t pinSourceTx = getPinSource(pinTx);
	
	uint8_t altFuncUsart = getUsartAltFunc(USARTx);
	
	/* Connect the given USART to the given pins */
	GPIO_PinAFConfig(bankPinRx, pinSourceRx, altFuncUsart); //Rx
	GPIO_PinAFConfig(bankPinTx, pinSourceTx, altFuncUsart); //Tx
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USARTx, &USART_InitStructure);
	
	USART_Cmd(USARTx, ENABLE); //Enable USARTx
	
	if(interruptFlag)
	{
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt when data is received
	}

}

uint8_t serial::read()
{
	return (USART_ReceiveData(thisUsart));
}

void serial::USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
}

void serial::write(uint8_t* packet, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		USART_puts(thisUsart, packet[i]);
	}
}

void serial::usartToClock(USART_TypeDef* USARTx)
{
  if (USARTx == USART1)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  }
  else if (USARTx == USART2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  }
  else if (USARTx == USART3)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  }    
  else if (USARTx == UART4)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  }
  else if (USARTx == UART5)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  }  
  else if (USARTx == USART6)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  }
  else if (USARTx == UART7)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
  }     
  else if(USARTx == UART8)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
  }
}

void serial::bankToClock(GPIO_TypeDef* bank)
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

uint16_t serial::getPinSource(uint16_t pin)
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

uint8_t serial::getUsartAltFunc(USART_TypeDef* USARTx)
{
	if(USARTx == USART1)
	{
		return GPIO_AF_USART1;
	}
	else if(USARTx == USART2)
	{
		return GPIO_AF_USART2;
	}
	else if(USARTx == USART3)
	{
		return GPIO_AF_USART3;
	}	
	else if(USARTx == UART4)
	{
		return GPIO_AF_UART4;
	}
	else if(USARTx == UART5)
	{
		return GPIO_AF_UART5;
	}
	else if(USARTx == USART6)
	{
		return GPIO_AF_USART6;
	}
	else if(USARTx == UART7)
	{
		return GPIO_AF_UART7;
	}
	else if(USARTx == UART8)
	{
		return GPIO_AF_UART8;
	}
}

