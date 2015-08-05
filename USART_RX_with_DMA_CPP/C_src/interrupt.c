#include "interrupt.h"

#include "stm32f4xx_conf.h"




void DMA1_Stream2_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
 
	for(int i = 0; i <  sizeof(Buffer); i++)
	  {
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET); // Wait for Empty
		USART_SendData(UART4, Buffer[i]);
	  }
	  
	  DMA_InitTypeDef  DMA_InitStructure;
  
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

	  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
  }

  
 
}