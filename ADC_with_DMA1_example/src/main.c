#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stm32f4xx_dma.h>
#include <stm32f4xx_adc.h>

//volatile uint8_t value = 8;

uint16_t ADC1ConvertedValue[8];

void init_USART1(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baud rate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pull up resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				  // the baud rate is set to the value we passed into this function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		  // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		  // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					  // again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	USART_Cmd(USART1, ENABLE);	//Enables USART1
}


//Functions that helps send information with USART
void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

void print16(uint16_t value)
{
	uint8_t thous = 0;
	uint8_t hund = 0;
	uint8_t tens = 0;
	uint8_t ones = 0; 
	
	uint16_t temp = value;
	while(temp >= 1000)
	{
		thous++;
		temp -= 1000; 
	}
	while(temp >= 100)
	{
		hund++;
		temp -= 100;
	}
	while(temp >= 10)
	{
		tens++;
		temp -= 10;
	}
	while(temp >= 1)
	{
		ones++;
		temp--;
	}
	
	USART_puts(USART1, (char)ones);
	USART_puts(USART1, (char)tens);
	USART_puts(USART1, (char)hund);
	USART_puts(USART1, (char)thous);
	
}

/*
 *	This function takes the name of the array to which you are storing the adc data. The next parameter is the size of the array that you are sending the data to. 
 *
 */

void initDma(uint16_t *array, uint16_t size)
{
	GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
 
  GPIO_StructInit(&GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  DMA_StructInit(&DMA_InitStructure);
 
  /**
    Set up the clocks are needed for the ADC
  */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
 
  /**
    Initialization of the GPIO Pins [OK]
  */
 
  /* Analog channel configuration : PC.01, 02*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /**
    Configure the DMA
  */
  //==Configure DMA2 - Stream 4
  DMA_DeInit(DMA2_Stream4);  //Set DMA registers to default values
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //Source address
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC1ConvertedValue; //Destination address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 8; //Buffer size
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure); //Initialize the DMA
  DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4
 
   /**
     Config the ADC1
   */
   ADC_DeInit();
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_NbrOfConversion = 8;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
   ADC_Init(ADC1,&ADC_InitStructure);
 
   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInit(&ADC_CommonInitStructure);
 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_144Cycles);//PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  2, ADC_SampleTime_144Cycles);//PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_144Cycles);//PC0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_144Cycles);//PC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_144Cycles);//PC4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 8, ADC_SampleTime_144Cycles);//PC5
 
   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
 
   ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA
 
   ADC_Cmd(ADC1, ENABLE);   // Enable ADC1
 
   ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion
}

void main(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configures the leds and the read/write pin on D1 */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 | GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	initDma(ADC1ConvertedValue, 8);
	init_USART1(9600);
	
	 /* GPIOD Periph clock enable */
	 RCC_AHB1PeriphClockCmd(USER_BUTTON_GPIO_CLK, ENABLE);

	 /* Configure Button in output pushpull mode */
	 GPIO_InitStructure.GPIO_Pin = USER_BUTTON_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStructure);
	
	uint8_t button = 0;
	uint8_t value = 0;
	uint8_t thousands = 0;
	
	uint8_t adc_value = 1;
	while(1)
	{
		
	    if(ADC1ConvertedValue[adc_value] > 4000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			
		}
		else if(ADC1ConvertedValue[adc_value] > 3000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
		}
		else if(ADC1ConvertedValue[adc_value] > 2000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
		}
		else if(ADC1ConvertedValue[adc_value] > 1000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);

		}
		else
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			GPIO_ResetBits(GPIOD, GPIO_Pin_13);
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		}
		
		
		/*button = GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);
		while(!button)
		{
			button = GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);
		}
		while(button)
		{
			button = GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN);
		}*/
		
		
		
			
	}
}