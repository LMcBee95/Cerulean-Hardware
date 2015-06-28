#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#define BUFFER_SIZE 	2
uint8_t Buffer[BUFFER_SIZE] = {1, 2};


void initLed(void);
void initButton(void);

void initUSART_DMA(void);
//void USART_puts(USART_TypeDef* USARTx, uint8_t data);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

void DMA1_Stream2_IRQHandler(void)
{
	USART_puts(UART4, Buffer[0]);
	
	//GPIO_SetBits(GPIOD, GPIO_Pin_15);
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		
	}
	else
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}
 
	/* Test on DMA Stream Half Transfer interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_HTIF2))
	{
		/* Clear DMA Stream Half Transfer interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_HTIF2);
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		
	}
	else
	{
		
	}
}

int main(void) {
	
	
	initLed();
	initButton();
	initUSART_DMA();
	
	int buttonPressed = 0;
	
	while (1)
	{  
		if( Buffer[0] == 0xff)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
		}
		else
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		}
		
		
		if(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN))
		{
			if(buttonPressed == 0)
			{
				for(int i = 0; i < 100; i++)
				{
					
				}
			}
			buttonPressed = 1;
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
		}
		else
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			buttonPressed = 0;
		}
		
	}
	return(0);
}


void initUSART_DMA(void)
{
	/* --------------------------- System Clocks Configuration -----------------*/
	/* UART4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
 
	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 
	/* DMA1 clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	
	  GPIO_InitTypeDef GPIO_InitStructure;
 
	  /*-------------------------- GPIO Configuration ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	  /* Connect USART pins to AF */
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
	  
	  
	   USART_InitTypeDef USART_InitStructure;
 
	  USART_InitStructure.USART_BaudRate = 9600;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 
	  USART_Init(UART4, &USART_InitStructure);
	 
	  USART_Cmd(UART4, ENABLE);
	  
	  
	  /** initiates the interupt handlers **/
	    NVIC_InitTypeDef NVIC_InitStructure;
 
	  /* Configure the Priority Group to 2 bits */
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 
	  /* Enable the UART4 RX DMA Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  
	  
	  
	  /** Configures the dma for the usart to store the rx data into a buffer automatically */
	  DMA_InitTypeDef  DMA_InitStructure;
 
	  //DMA_DeInit(DMA1_Stream2);
	 
	  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
	  
	  DMA_InitStructure.DMA_BufferSize = (uint16_t)BUFFER_SIZE;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	 
	 //sets the address location where the serial information will be storedss 
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
	  
	  
	  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	 
	  /* Enable the USART Rx DMA request */
	  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	 
	  /* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
	  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	  DMA_ITConfig(DMA1_Stream2, DMA_IT_HT, ENABLE);
	 
	  /* Enable the DMA RX Stream */
	  DMA_Cmd(DMA1_Stream2, ENABLE);
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
void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
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
