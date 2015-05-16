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

void init_DMA_ADC3(uint16_t *array, uint16_t size)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
 
  GPIO_StructInit(&GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);


 
  /**
    Set up the clocks are needed for the ADC
  */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

 
 /* Analog channel configuration : PF3, 4, 5, 10*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
 
   /**
     Config the ADC3
   */
   ADC_DeInit();
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //continuous conversion
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_NbrOfConversion = size;
   ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1=scan more that one channel in group
   ADC_Init(ADC3,&ADC_InitStructure);
 
   ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  1, ADC_SampleTime_144Cycles);//PF3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  2, ADC_SampleTime_144Cycles);//PF4
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  3, ADC_SampleTime_144Cycles);//PF5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_144Cycles);//PF10
   
 
  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}

void main(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configures the leds and the read/write pin on D1 */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	
	

	
	uint8_t adc_value = 0;
	while(1)
	{

		
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		
		Delay(0xffffff);
		
		
		GPIO_SetBits(GPIOD, GPIO_Pin_10);
		GPIO_SetBits(GPIOD, GPIO_Pin_11);
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		
		Delay(0xffffff);
	}
} 