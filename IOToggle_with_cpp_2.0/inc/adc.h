#ifndef __ADC_H
#define __ADC_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries
#include <string.h>


/* 				DMA Channels and streams associated with each adc			*/

/*					Channel					Stream
			
			ADC1:	DMA_Channel_0



*/


class adc
{
	public:
	
	adc(void);
	
	//functions to initiate the different ADC's
	void startAdc1(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc2(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc3(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);


	void addAdcPin(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum, const char* name);
	
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