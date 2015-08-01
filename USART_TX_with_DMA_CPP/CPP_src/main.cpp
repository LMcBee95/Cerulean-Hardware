#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"

#include "gpio.h"

DMA_InitTypeDef  DMA_InitStructure;

uint8_t Buffer[] = {1, 2, 3};


void initButton(void);

gpio green(GPIOD, GPIO_Pin_12);
	gpio orange(GPIOD, GPIO_Pin_13);
	gpio red(GPIOD, GPIO_Pin_14);
	gpio blue(GPIOD, GPIO_Pin_15);

void RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* UART4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
 
  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
 
  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC.10 UART4_TX, potential clash SCLK CS43L22
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
}
 
/**************************************************************************************/
 
void UART4_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
 
  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(UART4, &USART_InitStructure);
 
  USART_Cmd(UART4, ENABLE);
}
 
/**************************************************************************************/

 
void DMA_Configuration(void)
{
 
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(Buffer);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
 
  /* Enable the USART Tx DMA request */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
 
  /* Enable DMA Stream Transfer Complete interrupt */
  //DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
 
  /* Enable the DMA Tx Stream */
  DMA_Cmd(DMA1_Stream4, ENABLE);
}
 
/**************************************************************************************/
 
void DMA1_Stream4_IRQHandler(void)
{
	red.on();
	
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
  }
}
 
/**************************************************************************************/
 
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  /* Enable the UART4 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
 
/**************************************************************************************/
 



int main(void) {

	
	initButton();
	
	RCC_Configuration();
 
	//NVIC_Configuration();
 
	GPIO_Configuration();
 
	UART4_Configuration();
 
	DMA_Configuration();
	
	
	int buttonPressed = 0;
	
	while (1)
	{  
		
		if(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN))
		{
			if(buttonPressed == 0)
			{
				/* reinitializes the dma to trigger the sending again */
				DMA_DeInit(DMA1_Stream4);
				DMA_Init(DMA1_Stream4, &DMA_InitStructure);
				DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
				DMA_Cmd(DMA1_Stream4, ENABLE);
			
			}
			buttonPressed = 1;
			green.on();
		}
		else
		{
			green.off();
			buttonPressed = 0;
		}
		
		
		orange.on();
	}
	return(0);
}

void initUSART(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
 
	/* GPIOC clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
 
	 
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 
	  
	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
 
	GPIO_InitTypeDef GPIO_InitStructure;
 
	/*-------------------------- GPIO Configuration ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC.10 UART4_TX, potential clash SCLK CS43L22
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
	  /* Connect USART pins to AF */
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	  USART_InitTypeDef USART_InitStructure;
 
	  /* USARTx configuration ------------------------------------------------------*/
	  /* USARTx configured as follow:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	  */
	  USART_InitStructure.USART_BaudRate = 9600;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 
	  USART_Init(UART4, &USART_InitStructure);
	 
	  USART_Cmd(UART4, ENABLE);

}

void initDMA(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
 
	  //DMA_DeInit(DMA1_Stream4);
	 
	  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
	  DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(Buffer);
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
	 
	  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	 
	  /* Enable the USART Tx DMA request */
	  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	 
	  /* Enable DMA Stream Transfer Complete interrupt */
	  DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	 
	  /* Enable the DMA Tx Stream */
	  DMA_Cmd(DMA1_Stream4, ENABLE);
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

