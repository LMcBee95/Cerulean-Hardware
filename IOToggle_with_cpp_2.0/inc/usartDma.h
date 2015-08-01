#ifndef __USART_DMA_H
#define __USART_DMA_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

//Warning: the DMA write function deinitializes the entire stream, so if multiple DMA peripherals are activated on the same stream and you write with it, then it will uninitialize the remaing dma peripherals
//and only reinitialize the usart dma begin used


class usartDma
{
	public:
		
	/*
	* Function: constructor
	* Usage: usartDma myDma(USARTx);
	* 
	* ----------------------
	* Creates an instance of the usartDma class.
	* The tx and rx still need to be initialized
	*/
	usartDma(USART_TypeDef* USART_used);
	
	
	/*
	* Function: initializes tx with DMA
	* Usage: myDma.initTx(bufferSize, DMA_Stream, DmaChannel);
	* 
	* ----------------------
	* Initializes the tx DMA based on the specified size of packet, stream,
	* and channel.
	*/
	void initTx(uint8_t bufferSize, DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	
	/*
	* Function: sends the data stored in the buffer
	* Usage: myDma.write();
	* 
	* ----------------------
	* Sends out the data stored in the buffer. Warning, it writing deinitializes the stream, and 
	* any other peripherals initialized on the DMA stream will be deinitialized.
	*/
	
	void write(void);
	
	/*
	* Function: inserts data into the buffer at a specific location
	* Usage: myDma.insert(data, zero indexed location of data);
	* 
	* ----------------------
	* Stores data into the buffer that will be sent out by the DMA
	*/
	void insert(uint8_t data, uint8_t position);
		
	private:
		
	
	void getDmaClock(DMA_Stream_TypeDef* DMA_Stream);  //used to set the clock for the DMA based on which stream is being used
	
	USART_TypeDef* USARTx;  //used to store which usart is associated with this instance
	
	DMA_Stream_TypeDef* TxStream;  //used to store the stream that the tx is using
	
	uint8_t Buffer[75];  //buffer used to store the data that will be sent
	
	DMA_InitTypeDef  DMA_TX_InitStructure;  //the struct used to initialize the DMA channel
};



#endif