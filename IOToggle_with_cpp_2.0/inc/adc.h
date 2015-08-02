#ifndef __ADC_H
#define __ADC_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

class adc
{
	public:
	
	adc(void);
	
	//functions to initiate the different ADC's
	void startAdc1(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc2(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);
	void startAdc3(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel);


	void addAdcPin(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum);
	
	uint16_t get(uint8_t AdcUsed, uint8_t position);

	
	private: 

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