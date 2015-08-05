#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries
#include <string.h>


/* 				DMA Channels and streams associated with each adc
					(just copy and paste where you need them)

					Stream					Channel					
			
			ADC1:	DMA2_Stream0			DMA_Channel_0
					DMA2_Stream4			DMA_Channel_0

			ADC2: 	DMA2_Stream2			DMA_Channel_1
					DMA2_Stream3			DMA_Channel_1
					
			ADC3:	DMA2_Stream0			DMA_Channel_2
					DMA2_Stream1			DMA_Channel_2
*/


class adc
{
	public:
	
	/*
	* Function: constuctor
	* Usage: adc objName;
	* 
	* ----------------------
	* Creates an instance for the adc class. Only one instance should be made.
	*/
	adc(void);
	
	
	/*
	* Function: adds an adc pin
	* Usage: object.addAdcPin(ADCx, bank, pinNum, name)
	* 
	* ----------------------
	* Adds a pin to the adc. Each pin will have a name linked with it which will be used in the get
	* function to retreive its value.
	*/
	void addAdcPin(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum, const char* name);
	
	
	/*
	* Function: initialize Dma
	* Usage: object.startAdcX(stream, channel);
	* 
	* ----------------------
	*  Initializes the DMA after the pins of been initialized
	*/
	void startAdc1(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc2(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc3(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);

	
	/*
	* Function: retreive the adc value
	* Usage: object.get(name)
	* 
	* ----------------------
	* Retreives the value of the pin based on the name that is inputed.
	*
	* If a constant value of 1 or 0 is received, and not meant to, then the name that was inputed was 
	* speeled incorectly or does not exist.
	*/
	uint16_t get(const char* name);

	
	private: 

	//struct used to store the names associated with the location of each adc pin
	struct nameMaping
	{
		const char* name;
		uint8_t adcUsed;
		uint8_t numPosition;
	};
	
	nameMaping nameList[32];  //an array of name Mappings that will be used to retreive
	
	uint8_t nameListPosition = 0;  //position to store the next value in the nameList
	
	void getDmaClock(DMA_Stream_TypeDef* DMA_Stream);  //used to set the clock for the DMA based on which stream is being used
	
	void bankToClock(GPIO_TypeDef* bank);
	
	uint8_t getChannel(GPIO_TypeDef* bank, uint16_t pinNum);

	uint16_t data[3][16];
	
	uint8_t pinPriotity[3] = {1, 1, 1};
	
	DMA_InitTypeDef       DMA_ADC;
	
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;


};

#endif