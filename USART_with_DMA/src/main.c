#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#define PACKET_SIZE 	5

uint8_t packet[5] = {1, 2, 3, 4, 5};


void initLed(void);
void initButton(void);

void initUSART_DMA(void);
//void USART_puts(USART_TypeDef* USARTx, uint8_t data);

int main(void) {

	initLed();
	initButton();
	initUSART_DMA();
	
	int buttonPressed = 0;
	
	while (1)
	{  

		if(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN))
		{
			if(buttonPressed == 0)
			{
				
			}
			buttonPressed = 1;
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		}
		else
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			buttonPressed = 0;
		}
		
	
	}
	return(0);
}

/*void USART_puts(USART_TypeDef* USARTx, uint8_t data)
{
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}*/

void initUSART_DMA(void)
{
	DMA_InitTypeDef       DMA_Struct;
	
	// Turns on Peripheral Clocks for GPIO and USART functions
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// Configure GPIO Pins
	GPIO_InitTypeDef GPIO_Struct;

	// Configures GPIOA pins 9 and 10 for Serial AF
	GPIO_Struct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_Struct);

	// Setups Alternative Function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// Configure USART Peripheral
	USART_InitTypeDef USART_Struct;

	// Configure USART 1 settings
	USART_Struct.USART_BaudRate = 9600;
	USART_Struct.USART_WordLength = USART_WordLength_8b;
	USART_Struct.USART_StopBits = USART_StopBits_1;
	USART_Struct.USART_Parity = USART_Parity_No;
	USART_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Struct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_Struct);

	// Configure the Recieving Interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// Enables USART1
	USART_Cmd(USART1, ENABLE);

	// Configure NVIC
	NVIC_InitTypeDef NVIC_Struct;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_Struct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_Struct);

	// Configure DMA for USART
	//DMA_DeInit(DMA2_Stream7);

	DMA_Struct.DMA_Channel = DMA_Channel_4;
	DMA_Struct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_Struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_Struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Struct.DMA_Mode = DMA_Mode_Normal;
	DMA_Struct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_Struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_Struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Struct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_Struct.DMA_Priority = DMA_Priority_Low;

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);


	char* pointer = packet;
	int length = PACKET_SIZE;

	//DMA_DeInit(DMA2_Stream7);

	DMA_Struct.DMA_BufferSize = length;

	DMA_Struct.DMA_Memory0BaseAddr = (uint32_t)pointer;

	
	
	DMA_Init(DMA2_Stream7, &DMA_Struct);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA2_Stream7, ENABLE);
}


void initButton(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* GPIOD Periph clock enable */
	  RCC_AHB1PeriphClockCmd(USER_BUTTON_GPIO_CLK, ENABLE);

	  /* Configure Button in output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = USER_BUTTON_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStructure);
}

void initLed(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
