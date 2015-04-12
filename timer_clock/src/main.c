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

#define ONE_SECOND 1000

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
void TIM_Config(void);

uint32_t time = 0;
 
int main(void)
{
 TIM_Config();
 
 /* Time base configuration */
 TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //Frequency set to 1ms
 TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 
 /* TIM Interrupts enable */
 TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
 /* TIM3 enable counter */
 TIM_Cmd(TIM5, ENABLE);
 
while (1);
}
 
void TIM_Config(void)
{
 NVIC_InitTypeDef NVIC_InitStruct;
 
/* TIM3 clock enable */
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 
/* Enable the TIM3 gloabal Interrupt */
 NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
 NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStruct);
 
/* Initialize Leds mounted on STM324xG-EVAL board */
 STM_EVAL_LEDInit(LED3); //Initialize Orange LED
 STM_EVAL_LEDInit(LED4); //Initialize Green LED
 STM_EVAL_LEDInit(LED5); //Initialize Red LED
 STM_EVAL_LEDInit(LED6); //Initialize Blue LED 

}

void TIM5_IRQHandler(void)
{
 if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
 {
	
	if(time > ONE_SECOND * 20)
	{
	  STM_EVAL_LEDOn(LED6); //Blue LED
	}
	else if(time > ONE_SECOND * 15)
	{
		STM_EVAL_LEDOn(LED5); //Red LED
	}
	else if(time > ONE_SECOND * 10)
	{
		STM_EVAL_LEDOn(LED3); //Orange LED
	}
	else if(time > ONE_SECOND * 5)
	{
		STM_EVAL_LEDOn(LED4); //Green LED
	}

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	time++;
 }
}