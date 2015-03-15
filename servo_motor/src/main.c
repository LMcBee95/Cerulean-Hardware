/*
 *	This program creates pulse width modulation on TIM(er) 3 to run a servo motor. Pins C6: channel 1.   
 *	The frequency is 50 Hz, which means the period is is 20 ms, the same rate that servos take commands.
 *	Servos take a pwm pulse between 1 and 2 ms. 1 ms being the servo turned to 0 degrees and 2 ms meaning 
 *  the servo is turned to 180 degrees. The servo must receive a command every 24 or so ms or else the servo
 *  motor will disengage. 
 */

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 
#include <stm32f4xx_tim.h>  //library that contains all of the code the makes the pwm work


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/*This function initializes the pins that are used in pwm, assigns them to their alternate functions, 
 *and then initializes the TIM(ers) for those pins. For parameters, it takes the frequency of the pwm 
 *and the pre scaler for the clock. The frequency will work fine for anything bellow 525000, but it is 
 *not guaranteed to work above that. The pre scaler divides into the stm boards own internal clock to 
 *get a clock speed for the timer. The pre scaler works good at 1, but it can take any multiple of two 
 *as its input. The function will return the period of the pwm which is used to calculate the duty cycle 
 *when setting the pwm for each pin. 
 *
 *When deciding on the prescaller, use lower numbers for when you need higher frequencies and use higher
 *numbers when you need lower frequencies.  
 */
uint16_t initialize_timers(uint16_t frequency, uint16_t preScaler)
{
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);

	 
	// Compute prescaler value for timebase
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint16_t PreCalPeriod = ((84000000 / preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM3
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the pre scaler which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	 
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

void anologWrite(uint32_t channel, uint32_t period, uint8_t dutyCycle)
{
	channel = (period + 1) * dutyCycle / 255.0;
}

/*
 *	This function sets the angle of the servo motor. It takes the tim(er) and the second ardument is the channel 
 *  on that tim(er) that you want to move a servo on. The third argument is the angle you want to set the servo to.
 *  The servo varies from 0 to 180 degrees. The period of the servo will always be 50 Hz, and the variables MAX and 
 *  MIN should be adjusted to represented the pulse length in micro seconds that the pulse needs to be on for the servo
 *  to be at 0 degrees and 180 degrees respectively. 
 */

void setAngle(uint32_t channel, uint8_t angle, uint16_t period)
{
	uint16_t length = 1000 * angle / 180 + 100;  //1000 is the length of the max - min pulse length in u S. 180 is the full range of angles the servo motor can move. 500 is the minimum pulse length that corresponds with 0 degrees.
	
	TIM3->CCR1 = (period + 1) * 2.25 / 20;  //Sets the pulse width length to be length micro Seconds
}


int main(void)
{
	uint16_t period = initialize_timers(50, 64);
	
	//setAngle(TIM3->CCR1, 90, period);  //Sets the angle of the servo attached to pin C6 to 90 degrees
	
	uint32_t max = 2.25;  //max length of pulse in mili seconds that the servo will take
	uint32_t min = 0.75;  //min length of pulse in mili seconds that the servo will take
	
	while(1)
	{
		TIM3->CCR1 = (period + 1) * 0.75 / 20;
		
		Delay(0xFFFFFF);
		
	}
}