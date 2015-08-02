#include "adc.h"

adc::adc(void)
{
	//sets up part of the dma portion of the initialization, the remaining parts are initialized with the specific adc
	DMA_ADC.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_ADC.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_ADC.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_ADC.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
	DMA_ADC.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
	DMA_ADC.DMA_Mode = DMA_Mode_Circular;
	DMA_ADC.DMA_Priority = DMA_Priority_High;
	DMA_ADC.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_ADC.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_ADC.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_ADC.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	//sets up the adc portion of the initialization
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

}


void adc::startAdc1(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel)
{	
	uint8_t size = pinPriotity[0] - 1;
	
	getDmaClock(DMA_Stream);  //initializes the dma clock associated with this streamS
	
	DMA_ADC.DMA_Channel = DmaChannel;  //sets the dma channel
	DMA_ADC.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //source of the adc data
	DMA_ADC.DMA_Memory0BaseAddr = (uint32_t)data[0]; //Destination of the adc data
	DMA_ADC.DMA_BufferSize = size; //how many values that are being read in
	
	DMA_Init(DMA_Stream, &DMA_ADC); //Initialize the DMA
	DMA_Cmd(DMA_Stream, ENABLE); 
 
	
	ADC_InitStructure.ADC_NbrOfConversion = size;
	ADC_Init(ADC1,&ADC_InitStructure);
 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

}

void adc::startAdc2(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel)
{
	uint8_t size = pinPriotity[1] - 1;
	
	getDmaClock(DMA_Stream);
	
	DMA_ADC.DMA_Channel = DmaChannel;
	DMA_ADC.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;
	DMA_ADC.DMA_Memory0BaseAddr = (uint32_t)&data[1];
	DMA_ADC.DMA_BufferSize = size;
	
	DMA_Init(DMA_Stream, &DMA_ADC);
	DMA_Cmd(DMA_Stream, ENABLE); 
    
	
	ADC_InitStructure.ADC_NbrOfConversion = size;
	ADC_Init(ADC2,&ADC_InitStructure);
	
	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
	ADC_DMACmd(ADC2, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_SoftwareStartConv(ADC2);
}

void adc::startAdc3(DMA_Stream_TypeDef* DMA_Stream, uint32_t DmaChannel)
{
	uint8_t size = pinPriotity[2] - 1;
	
	getDmaClock(DMA_Stream);
	
	DMA_ADC.DMA_Channel = DmaChannel;
	DMA_ADC.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;
	DMA_ADC.DMA_Memory0BaseAddr = (uint32_t)&data[2];
	DMA_ADC.DMA_BufferSize = size;
	
	DMA_Init(DMA_Stream, &DMA_ADC);
	DMA_Cmd(DMA_Stream, ENABLE); 
	
	
	ADC_InitStructure.ADC_NbrOfConversion = size;
	ADC_Init(ADC3,&ADC_InitStructure);
	
	
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC3);
}

void adc::addAdcPin(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum)
{
	
	//structures used in the initialization of each type
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	bankToClock(bank);  //initiates the clcok associated with the bank
	
	  GPIO_InitStructure.GPIO_Pin   = pinNum;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(bank, &GPIO_InitStructure);
	
	uint8_t position = 0;  //variable used to tell what order to read in the adc pins
	
	if(ADCx == ADC1)
		position = pinPriotity[0]++;
	else if(ADCx == ADC2)
		position = pinPriotity[1]++;
	else if(ADCx == ADC3)
		position = pinPriotity[2]++;
	
	//Adds the pin to the ADC
	ADC_RegularChannelConfig(ADCx, getChannel(bank, pinNum),  position, ADC_SampleTime_144Cycles);
	
}

uint16_t adc::get(uint8_t AdcUsed, uint8_t position)
{
	return data[AdcUsed - 1][position];
}

uint8_t adc::getChannel(GPIO_TypeDef* bank, uint16_t pinNum)
{
	if(bank == GPIOA)
	{
		if (pinNum == GPIO_Pin_0)
			return ADC_Channel_0;
		
		else if (pinNum == GPIO_Pin_1)
			return ADC_Channel_1;
		
		else if (pinNum == GPIO_Pin_2)
			return ADC_Channel_2;
		
		else if (pinNum == GPIO_Pin_3)
			return ADC_Channel_3;
		
		else if (pinNum == GPIO_Pin_4)
			return ADC_Channel_4;
		
		else if (pinNum == GPIO_Pin_5)
			return ADC_Channel_5;
		
		else if (pinNum == GPIO_Pin_6)
			return ADC_Channel_6;
		
		else if (pinNum == GPIO_Pin_7)
			return ADC_Channel_7;
	}
	else if(bank == GPIOB)
	{
		if (pinNum == GPIO_Pin_0)
			return ADC_Channel_8;
		
		else if (pinNum == GPIO_Pin_1)
			return ADC_Channel_9;
	}	
	else if(bank == GPIOC)
	{
		if (pinNum == GPIO_Pin_0)
			return ADC_Channel_10;
		
		else if (pinNum == GPIO_Pin_1)
			return ADC_Channel_11;
		
		else if (pinNum == GPIO_Pin_2)
			return ADC_Channel_12;
		
		else if (pinNum == GPIO_Pin_3)
			return ADC_Channel_13;
		
		else if (pinNum == GPIO_Pin_4)
			return ADC_Channel_14;
		
		else if (pinNum == GPIO_Pin_5)
			return ADC_Channel_15;
	}
	else if(bank == GPIOF)
	{
		if (pinNum == GPIO_Pin_6)
			return ADC_Channel_4;
		
		else if (pinNum == GPIO_Pin_7)
			return ADC_Channel_5;
		
		else if (pinNum == GPIO_Pin_8)
			return ADC_Channel_6;
		
		else if (pinNum == GPIO_Pin_9)
			return ADC_Channel_7;
		
		else if (pinNum == GPIO_Pin_10)
			return ADC_Channel_8;
		
		else if (pinNum == GPIO_Pin_3)
			return ADC_Channel_9;
		
		else if (pinNum == GPIO_Pin_4)
			return ADC_Channel_14;
		
		else if (pinNum == GPIO_Pin_5)
			return ADC_Channel_15;
	}
	
	
	return ADC_Channel_1;  //if the code reaches this point then the user has inputed the wrong values
}

void adc::getDmaClock(DMA_Stream_TypeDef* DMA_Stream)
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

void adc::bankToClock(GPIO_TypeDef* bank)
{
	
	//initializes the correct clock that goes with the inputed bank
	if (bank == GPIOA)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	}
	else if (bank == GPIOB)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	}
	else if (bank == GPIOC)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  	}
	else if (bank == GPIOD)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	}
	else if (bank == GPIOE)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	}
	else if (bank == GPIOF)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	}
	else if (bank == GPIOG)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	}
}