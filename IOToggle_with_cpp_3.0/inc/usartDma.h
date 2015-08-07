#ifndef __USART_DMA_H
#define __USART_DMA_H

#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

//Warning: Make sure that no Tx dma uses the same stream as an Rx dma or else each one will get 
//reset when the other tries to get data


//Not sure if the priority in the NVIC should be different or if all of them can be the same



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
	* Usage: myDma.send();
	* 
	* ----------------------
	* Sends out the data stored in the buffer. Warning, it writing deinitializes the stream, and 
	* any other peripherals initialized on the DMA stream will be deinitialized.
	*/
	
	void send(void);
	
	/*
	* Function: inserts data into the buffer at a specific location
	* Usage: myDma.insert(data, zero indexed location of data);
	* 
	* ----------------------
	* Stores data into the buffer that will be sent out by the DMA
	*/
	void insert(uint8_t data, uint8_t position);
	
	
	
	uint8_t get(uint8_t position);
	
	
	void initRx(uint8_t bufferSize);
		
	private:
		
	/* Private Functions */
	
	uint32_t getChannel();  //gets the chanel associated with the rx Dma
	
	uint8_t getDmaNum();  //Gets the Dma number that the Rx is in and then 0 indexes it to work witht he array
	
	uint8_t getStreamNum();  //returns the stream that is used by the rx for Dma. NOT ZERO INDEXED
	
	DMA_Stream_TypeDef* getRxStream();  //gets the correct stream for the Rx based on the Usart. No inputs needed
	
	void getDmaClock(DMA_Stream_TypeDef* DMA_Stream);  //used to set the clock for the DMA based on which stream is being used
	
	void NVIC_Configuration();  //configures the interrupt handleer for the rx for dma
	
	
	
	/* Private Instance Variables */
	
	
	USART_TypeDef* USARTx;  //used to store which usart is associated with this instance
	
	DMA_Stream_TypeDef* TxStream;  //used to store the stream that the tx is using
	
	uint8_t Buffer[75];  //buffer used to store the data that will be sent
	
	uint8_t RxBuffer[50] = {0};
	
	DMA_InitTypeDef  DMA_TX_InitStructure;  //the struct used to initialize the DMA channel
	
	DMA_InitTypeDef  DMA_RX_InitStructure;  //the struct used to initialize the DMA channel
	
	uint8_t streamNum;  //stores the stream used in number form to be with indexing in an array
	
	uint8_t channelNum;  //a zero indexed number of the Dma number that the rx uses
	
	uint8_t priority = 0;  //used to give each rx dma interrupt a different priority
};



#endif