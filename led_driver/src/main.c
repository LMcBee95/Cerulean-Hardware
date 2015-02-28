#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_dma.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_tim.h>

volatile uint8_t value = 7;

uint16_t ADC3ConvertedValue[7];


/*This function initializes the pins that are used in pwm, assigns them to their alternate functions, 
 *and then initializes the TIM(ers) for those pins. For parameters, it takes the frequency of the pwm 
 *and the prescaler for the clock. The frequency will work fine for anything bellow 525000, but it is 
 *not guaranteed to work above that. The prescaler divides into the stm boards own internal clock to 
 *get a clock speed for the timer. The prescaler works good at 1, but it can take any multiple of two 
 *as its input. The function will return the period of the pwm which is used to calculate the duty cycle 
 *when setting the pwm for each pin 
 */
int32_t initialize_timers(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM3
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	 
	// Enable TIM3 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	 
	// Enable TIM3 counter
	TIM_Cmd(TIM3, ENABLE); 
	
	return(PreCalPeriod);
}




void initDma(void)
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
    DMA_InitStruct.DMA_BufferSize = value;
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
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;//PA1 , PA2, PA3
    GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration
	
    
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
    ADC_InitStruct.ADC_NbrOfConversion = value;
    ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration
    
	/* Select the channels to be read from **************************************/
    
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  1, ADC_SampleTime_144Cycles);//PA1
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2,  2, ADC_SampleTime_144Cycles);//PA2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  3, ADC_SampleTime_144Cycles);//PA3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_144Cycles);//PC0
    ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 5, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 6, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 7, ADC_SampleTime_144Cycles);//PC3
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);
    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);
}

/*
 *This function controls the voltage outputted by each pwm pin. For parameters, it takes the TIM(er) of the 
 *pin that is being changed, which CCRx (Constant current reduction) should be selected, the period of the duty 
 *cycle (which can be obtained from the initialization function), and the desired duty cycle (between 0 and 255).
 */

void anologWrite(uint32_t channel, uint8_t dutyCycle, uint32_t period)
{
	channel = (period + 1) * dutyCycle / 255.0;
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
	
	initDma();
	
	uint32_t period = initialize_timers(525000, 1);
	
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
	while(1)
	{
		
	    TIM3->CCR1 = (period + 1) * ADC3ConvertedValue[0] / 4000;
		
		/*if(ADC3ConvertedValue[0] > 1000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			TIM3->CCR1 = (period + 1) * 50.0 / 100;
		}
		else if(ADC3ConvertedValue[1] > 1000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			TIM3->CCR2 = (period + 1) * 50.0 / 100;
		}
		else if(ADC3ConvertedValue[2] > 1000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			TIM3->CCR3 = (period + 1) * 50.0 / 100;
		}
		else if(ADC3ConvertedValue[3] > 1000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			TIM3->CCR4 = (period + 1) * 50.0 / 100;
		}
		else
		{
			TIM3->CCR1 = (period + 1) * 0 / 100;
			TIM3->CCR2 = (period + 1) * 0 / 100;
			TIM3->CCR3 = (period + 1) * 0 / 100;
			TIM3->CCR4 = (period + 1) * 0 / 100;
		}*/
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			
	}
}