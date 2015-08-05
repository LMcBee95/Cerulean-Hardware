// STM32 UART4 DMA RX (Tx PA.0, Rx PA.1) STM32F4 Discovery - sourcer32@gmail.com
 
#include "stm32f4_discovery.h"
 
 
 
 void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
 
   DMA_InitTypeDef  DMA_InitStructure;
 
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
 
uint8_t Buffer[1];
 
void DMA_Configuration(void)
{
 
  DMA_DeInit(DMA1_Stream2);
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
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
 
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
 
  /* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMA1_Stream2, ENABLE);
}
 
/**************************************************************************************/
 
void DMA1_Stream2_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
 
    USART_SendData(UART4, Buffer[0]);

	  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
  }
 
  Delay(0xffff);
 
		
	DMA_DeInit(DMA1_Stream2);
 
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
 
  /* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMA1_Stream2, ENABLE);

  
 
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
    RCC_Configuration();
 
  NVIC_Configuration();
 
    GPIO_Configuration();
 
  UART4_Configuration();
 
  DMA_Configuration();
 
  while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
  USART_SendData(UART4, 1);
 
  while(1); // Don't want to exit
}