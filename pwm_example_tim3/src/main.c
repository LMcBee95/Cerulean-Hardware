/*
 *	This program creates pulse width modulation on TIM(er) 3. Pins C6: channel 1, C7: channel 2, C8 channel 3, and C9 channel 4. 
 *	To set the duty cycle of the specific pin, TIMx->CCRy = duty cycle, where x is the TIM(er) that you are using and y is the 
 *	channel within the TIM(er) that you want to use. 
 *
 */

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 
#include <stm32f4xx_tim.h>  //library that contains all of the code the makes the pwm work


/*This function initializes the pins that are used in pwm, assigns them to their alternate functions, 
 *and then initializes the TIM(ers) for those pins. For parameters, it takes the frequency of the pwm 
 *and the prescaler for the clock. The frequency will work fine for anything bellow 525000, but it is 
 *not guaranteed to work above that. The prescaler divides into the stm boards own internal clock to 
 *get a clock speed for the timer. The prescaler works good at 1, but it can take any multiple of two 
 *as its input. The function will return the period of the pwm which is used to calculate the duty cycle 
 *when setting the pwm for each pin 
 */
int32_t initialize_pwm_timers(uint32_t frequency, uint16_t preScaler)
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

/*
 *This function controls the voltage outputted by each pwm pin. For parameters, it takes the TIM(er) of the 
 *pin that is being changed, which CCRx (Constant current reduction) should be selected, the period of the duty 
 *cycle (which can be obtained from the initialization function), and the desired duty cycle (between 0 and 255).
 */

void anologWrite(uint32_t channel, uint8_t dutyCycle, uint32_t period)
{
	channel = (period + 1) * dutyCycle / 255.0;
}


int main(void)
{
	uint32_t period = initialize_pwm_timers(525000, 1);
	
	while(1)
	{
			//anologWrite(TIM3->CCR1, 125.0, period) ; //Sets the dury cycle of the pulse width modulation. The duty cycle is how long the pin stay on during a cycle.
			TIM3->CCR1 = (period + 1) * 125.0 / 255.0;
			//So the higher the duty cycle, up until the max duty cycle, the higher the voltage the pulse width will emit.
			//sets the duty cycle of the other pins
			TIM3->CCR2 = 40; 
			TIM3->CCR3 = 40; 
			TIM3->CCR4 = 40; 
	}
}