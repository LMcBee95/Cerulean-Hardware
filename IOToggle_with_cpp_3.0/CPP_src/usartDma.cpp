#include "usartDma.h"

#include "DMA.h"


  //links in the c file that contains all of the interrupts.
					//the interupts MUST BE IN A .c FILE, not a .cpp file, or else it will not work



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

void usartDma::send(void)
{
	DMA_DeInit(TxStream);
	DMA_Init(TxStream, &DMA_TX_InitStructure);
	DMA_ITConfig(TxStream, DMA_IT_TC, ENABLE);
	DMA_Cmd(TxStream, ENABLE);
}

uint8_t usartDma::get(uint8_t position)
{	
	return RxBuffer[position];
}

void usartDma::initRx(uint8_t bufferSize)
{
	
	DMA_Stream_TypeDef* RxStream = getRxStream();
	uint32_t RxChannel = getChannel();
	
	
	getDmaClock(RxStream); 
	
	streamNum = getStreamNum(); 
	
	channelNum = getDmaNum();
	
	NVIC_Configuration();
	

	
	//initializes the dma to send information from a buffer in normal mode
	RxStruct[channelNum][streamNum].DMA_Channel = RxChannel; //the dma chanel that is selected
	RxStruct[channelNum][streamNum].DMA_DIR = DMA_DIR_PeripheralToMemory;
	RxStruct[channelNum][streamNum].DMA_Memory0BaseAddr = (uint32_t)RxBuffer; 
	RxStruct[channelNum][streamNum].DMA_BufferSize = (uint16_t) bufferSize;
	RxStruct[channelNum][streamNum].DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;
	RxStruct[channelNum][streamNum].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	RxStruct[channelNum][streamNum].DMA_MemoryInc = DMA_MemoryInc_Enable;
	RxStruct[channelNum][streamNum].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	RxStruct[channelNum][streamNum].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	RxStruct[channelNum][streamNum].DMA_Mode = DMA_Mode_Normal;
	RxStruct[channelNum][streamNum].DMA_Priority = DMA_Priority_High;
	RxStruct[channelNum][streamNum].DMA_FIFOMode = DMA_FIFOMode_Enable;
	RxStruct[channelNum][streamNum].DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	RxStruct[channelNum][streamNum].DMA_MemoryBurst = DMA_MemoryBurst_Single;
	RxStruct[channelNum][streamNum].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
	DMA_Init(RxStream, &RxStruct[channelNum][streamNum]);
 
	
	USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
 
	
	DMA_ITConfig(RxStream, DMA_IT_TC, ENABLE);
 
	
	DMA_Cmd(RxStream, ENABLE);
	
	
	
}

uint8_t usartDma::getDmaNum()
{
	if(USARTx == USART1)
	{
		return 1;
	}
	else if(USARTx == USART2)
	{
		return 0;
	}
	else if(USARTx == USART3)
	{
		return 0;
	}
	else if(USARTx == UART4)
	{
		return 0;
	}
	else if(USARTx == UART5)
	{
		return 0;
	}
	else if(USARTx == USART6)
	{
		return 1;
	}
	else if(USARTx == UART7)
	{
		//Most chips don't support this UART so I won't do it
	}
	else if(USARTx == UART8)
	{
		//Most chips don't support this UART so I won't do it
	}
	
	return 0;
}

uint8_t usartDma::getStreamNum()
{
	if(USARTx == USART1)
	{
		return 5;
	}
	else if(USARTx == USART2)
	{
		return 5;
	}
	else if(USARTx == USART3)
	{
		return 1;
	}
	else if(USARTx == UART4)
	{
		return 2;
	}
	else if(USARTx == UART5)
	{
		return 0;
	}
	else if(USARTx == USART6)
	{
		return 1;
	}
	else if(USARTx == UART7)
	{
		//Most chips don't support this UART so I won't do it now
	}
	else if(USARTx == UART8)
	{
		//Most chips don't support this UART so I won't do it now
	}
	
	return 0;
}

DMA_Stream_TypeDef* usartDma::getRxStream()
{
	//Matching the USART with the correct stream to use. Each stream has an alternate but 
	//if much too confusing to allow the user to choose, so if you need to choose another stream
	//due to conflicts with other streams, feel free to comment out any of these and change them
	if(USARTx == USART1)
	{
		return DMA2_Stream5;
	}
	else if(USARTx == USART2)
	{
		return DMA1_Stream5;
	}
	else if(USARTx == USART3)
	{
		return DMA1_Stream1;
	}
	else if(USARTx == UART4)
	{
		return DMA1_Stream2;
	}
	else if(USARTx == UART5)
	{
		return DMA1_Stream0;
	}
	else if(USARTx == USART6)
	{
		return DMA2_Stream1;
	}
	else if(USARTx == UART7)
	{
		//Most chips don't support this UART so I won't do it now
	}
	else if(USARTx == UART8)
	{
		//Most chips don't support this UART so I won't do it now
	}
	
	return DMA1_Stream2;  //a default value that will be return if the USART cannot be found
}

uint32_t usartDma::getChannel()
{
	if(USARTx == USART1)
	{
		return DMA_Channel_4;
	}
	else if(USARTx == USART2)
	{
		return DMA_Channel_4;
	}
	else if(USARTx == USART3)
	{
		return DMA_Channel_4;
	}
	else if(USARTx == UART4)
	{
		return DMA_Channel_4;
	}
	else if(USARTx == UART5)
	{
		return DMA_Channel_4;
	}
	else if(USARTx == USART6)
	{
		return DMA_Channel_5 ;
	}
	else if(USARTx == UART7)
	{
		//Most chips don't support this UART so I won't do it now
	}
	else if(USARTx == UART8)
	{
		//Most chips don't support this UART so I won't do it now
	}
	
	
	return 0;
}

void usartDma::NVIC_Configuration()
{
	NVIC_InitTypeDef NVIC_InitStructure;
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 

	if(USARTx == USART1)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	}
	else if(USARTx == USART2)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	}
	else if(USARTx == USART3)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	}
	else if(USARTx == UART4)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	}
	else if(USARTx == UART5)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	}
	else if(USARTx == USART6)
	{
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	}
	else if(USARTx == UART7)
	{
		//Most chips don't support this UART so I won't do it now
	}
	else if(USARTx == UART8)
	{
		//Most chips don't support this UART so I won't do it now
	}

	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = priority++;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

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
