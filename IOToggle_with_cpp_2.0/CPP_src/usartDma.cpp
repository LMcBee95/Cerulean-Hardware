#include "usartDma.h"

usartDma::usartDma(USART_TypeDef* USART_used)
{
	USARTx = USART_used;
}

void usartDma::initTx(uint8_t bufferSize, DMA_Stream_TypeDef* usedStream, uint32_t DmaChannel)
{
	TxStream = usedStream;  //gets the stream from the user and stores it into a global instance variable

	
	getDmaClock(TxStream);
	
	
	//initializes the dma to send information from a buffer in normal mode
	DMA_TX_InitStructure.DMA_Channel = DmaChannel; //the dma chanel that is selected
	DMA_TX_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_TX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Buffer;
	DMA_TX_InitStructure.DMA_BufferSize = (uint16_t) bufferSize;
	DMA_TX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;
	DMA_TX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_TX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_TX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_TX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_TX_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_TX_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_TX_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_TX_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_TX_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_TX_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
	DMA_Init(TxStream, &DMA_TX_InitStructure);
 
	/* Enable the USART Tx DMA request */
	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
 
	/* Enable DMA Stream Transfer Complete interrupt */
	//DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
 
	/* Enable the DMA Tx Stream */
	DMA_Cmd(TxStream, ENABLE);
}

void usartDma::write(void)
{
	DMA_DeInit(TxStream);
	DMA_Init(TxStream, &DMA_TX_InitStructure);
	DMA_ITConfig(TxStream, DMA_IT_TC, ENABLE);
	DMA_Cmd(TxStream, ENABLE);
}

void usartDma::insert(uint8_t data, uint8_t position)
{
	Buffer[position] = data;
}




void usartDma::getDmaClock(DMA_Stream_TypeDef* DMA_Stream)
{
	if(DMA_Stream == DMA1_Stream0 || DMA_Stream == DMA1_Stream1 || DMA_Stream == DMA1_Stream2 || DMA_Stream == DMA1_Stream3 || DMA_Stream == DMA1_Stream4
								|| DMA_Stream == DMA1_Stream5 || DMA_Stream == DMA1_Stream6 ||  DMA_Stream == DMA1_Stream7) 
	{
		 /* DMA1 clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	}
	else if(DMA_Stream == DMA2_Stream0 || DMA_Stream == DMA2_Stream1 || DMA_Stream == DMA2_Stream2 || DMA_Stream == DMA2_Stream3 || DMA_Stream == DMA2_Stream4 
								       || DMA_Stream == DMA2_Stream5 || DMA_Stream == DMA2_Stream6||  DMA_Stream == DMA2_Stream7) 
	{
		/* DMA2 clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	}
		
	
}
