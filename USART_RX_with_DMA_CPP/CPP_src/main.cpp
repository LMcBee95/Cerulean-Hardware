// STM32 UART4 DMA RX (Tx PA.0, Rx PA.1) STM32F4 Discovery - sourcer32@gmail.com
 
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "gpio.h"
 
 
 gpio green(GPIOD, GPIO_Pin_12);
gpio orange(GPIOD, GPIO_Pin_13);
gpio red(GPIOD, GPIO_Pin_14);
gpio blue(GPIOD, GPIO_Pin_15);

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
 
/**************************************************************************************/
 
void RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* UART4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
 
  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 
  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
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
}
 
/**************************************************************************************/
 
void UART4_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
 
  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 4800 baud
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
 
uint8_t Buffer[4] = {0};
 
void DMA_Configuration(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
 
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
  DMA_InitStructure.DMA_BufferSize = 4;
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
 
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
 
  /* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
  //DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
  //DMA_ITConfig(DMA1_Stream2, DMA_IT_HT, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMA1_Stream2, ENABLE);
}
 
/**************************************************************************************/
 
void DMA1_Stream2_IRQHandler(void)
{
	green.on();

	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF1))
	{
		// ----------
	
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF1);
		Delay(0xfff);
	}
}
 
/**************************************************************************************/
 
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  /* Enable the UART4 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
 
/**************************************************************************************/
 



int main(void)
{
	blue.on();
	
RCC_Configuration();
 
  NVIC_Configuration();
 
    GPIO_Configuration();
 
  UART4_Configuration();
 
  DMA_Configuration();
 

	
	red.on();
 
  while(1)
  {
	/*while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(UART4, Buffer[0]);
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(UART4, Buffer[1]);
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(UART4, Buffer[2]);
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(UART4, Buffer[3]);*/
	  
	 // Delay(0xfffff3);
	  
	if(Buffer[0] == 1 && Buffer[1] == 2 && Buffer[2] == 3 && Buffer[3] == 4 )
		blue.on();
	else
		blue.off();
  }
	
}