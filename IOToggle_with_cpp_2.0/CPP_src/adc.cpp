#include "adc.h"

DMA_InitTypeDef       DMA_ADC1;
DMA_InitTypeDef       DMA_ADC2;
DMA_InitTypeDef       DMA_ADC3;

ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;

uint16_t adc1[16];
uint16_t adc2[16];
uint16_t adc3[16];

void startAdc1(uint8_t size)
{	
	DMA_ADC1.DMA_Channel = DMA_Channel_0;
	DMA_ADC1.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //Source address
	DMA_ADC1.DMA_Memory0BaseAddr = (uint32_t)adc1; //Destination address
	DMA_ADC1.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_ADC1.DMA_BufferSize = size; //Buffer size
	DMA_ADC1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_ADC1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_ADC1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
	DMA_ADC1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
	DMA_ADC1.DMA_Mode = DMA_Mode_Circular;
	DMA_ADC1.DMA_Priority = DMA_Priority_High;
	DMA_ADC1.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_ADC1.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_ADC1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_ADC1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_ADC1); //Initialize the DMA
	DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4
 
	/**
	* Config the ADC1 
	**/
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = size;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
	ADC_Init(ADC1,&ADC_InitStructure);
 
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);
}

void startAdc2(void)
{

}

void startAdc3(uint8_t size)
{
	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_ADC3.DMA_Channel = DMA_Channel_2;
	DMA_ADC3.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
	DMA_ADC3.DMA_Memory0BaseAddr = (uint32_t)&adc3;
	DMA_ADC3.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_ADC3.DMA_BufferSize = size;
	DMA_ADC3.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_ADC3.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_ADC3.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
	DMA_ADC3.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
	DMA_ADC3.DMA_Mode = DMA_Mode_Circular;
	DMA_ADC3.DMA_Priority = DMA_Priority_High;
	DMA_ADC3.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_ADC3.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_ADC3.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_ADC3.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_ADC3);
	DMA_Cmd(DMA2_Stream0, ENABLE); 
    
	/**
	* Config the ADC1 
	**/
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = size;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
	ADC_Init(ADC1,&ADC_InitStructure);
 
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);
}

void addAdc(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum)
{
	//structures used in the initialization of each type
	GPIO_InitTypeDef      GPIO_InitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	/*GPIO_StructInit(&GPIO_InitStructure);
	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	DMA_StructInit(&DMA_InitStructure);*/
	
	bankToClock(bank);  //initiates the clcok associated with the bank
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);  //initiates the DMA2 clock
	
	//initiates the clock for the specified ADC
	if(ADCx == ADC1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	else if(ADCx == ADC2)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	else if(ADCx == ADC3)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	
	 /* Alternate function for ADC configuration */
	  GPIO_InitStructure.GPIO_Pin   = pinNum;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(bank, &GPIO_InitStructure);
}

void bankToClock(GPIO_TypeDef* bank)
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