#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stm32f4xx_dma.h>
#include <stm32f4xx_adc.h>


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

uint16_t ADC3ConvertedValue[4] = {0,0,0,0};

void initDMA(void)
{
	ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;
    GPIO_InitTypeDef      GPIO_InitStruct;
    
	
	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//ADC3 is connected to the APB2 peripheral bus
    
	
	
	/* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStruct.DMA_Channel = DMA_Channel_2;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = 4;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream0, ENABLE);
   
   
   /* Configure GPIO pins ******************************************************/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;// PC0, PC1, PC2, PC3
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down
    GPIO_Init(GPIOC, &GPIO_InitStruct);//Initialize GPIOC pins with the configuration
    //GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PA1
    //GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration
    
    
    /* ADC Common Init **********************************************************/
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);
   


/* ADC3 Init ****************************************************************/
    ADC_DeInit();
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = 4;
    ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration
    /* Select the channels to be read from **************************************/
    
    
    
    ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);//PC0
    ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);//PC3
    //ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  5, ADC_SampleTime_144Cycles);//PA1
    
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
   

    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);
    
    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
    
    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);	
}

void initDMA1(uint8_t size)
{
	ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;
    GPIO_InitTypeDef      GPIO_InitStruct;
    
	
	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//ADC3 is connected to the APB2 peripheral bus
    
	/* Analog channel configuration : PF3, 4, 5, 10*/
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	
	

	
	/* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStruct.DMA_Channel = DMA_Channel_2;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = size;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream0, ENABLE);
   
   
   /* Configure GPIO pins *****************************************************
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;// PC0, PC1, PC2, PC3
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down
    GPIO_Init(GPIOC, &GPIO_InitStruct);//Initialize GPIOC pins with the configuration
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PA1
    GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration */
    
    
    /* ADC Common Init **********************************************************/
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);
   


/* ADC3 Init ****************************************************************/
    ADC_DeInit();
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = size;
    ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration
    /* Select the channels to be read from **************************************/
    
    
    
    /*ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);//PC0
    ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);//PC3
    ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  5, ADC_SampleTime_144Cycles);//PA1*/
    
    
    ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  1, ADC_SampleTime_144Cycles);//PF3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_9,  2, ADC_SampleTime_144Cycles);//PF4
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14,  3, ADC_SampleTime_144Cycles);//PF5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 4, ADC_SampleTime_144Cycles);//PF10
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
   

    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);
    
    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
    
    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);
}


int main(void) 
{
	initDMA1(4);
	//zinitDMA();
    

    
    GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	  
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configures the leds and the read/write pin on D1 */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	

	
	uint8_t adc_value = 0;
    
    
    //GPIO_SetBits(GPIOD, GPIO_Pin_13);
    
    while(1) 
    {
	    
	    Delay(0xffff);

		if(ADC3ConvertedValue[0] > 2000   )
		{
	    
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
		}	
		else
	
		{
	    
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);

		}
    } 
    return 1;
}

