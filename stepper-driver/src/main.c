/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "stepper.h"

#include <stdlib.h>
#include <math.h>

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
void Beep(int beeps, GPIO_TypeDef* beepBlock, uint16_t beepPin);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  //Initialize a stepper structure, with the step pin set to pin 12, the direction
  //pin set to pin 13, and the enable pin set to 14
  Stepper* verticalStepper = Stepper_Initialize(
	GPIOD, GPIO_Pin_12,
	GPIOD, GPIO_Pin_13,
	GPIOD, GPIO_Pin_14, 1);
	
  //Initialize another stepper structure, with the step pin set to pin 4, the direction
  //pin set to pin 5, and the enable pin set to 6
  Stepper* horizontalStepper = Stepper_Initialize(
    GPIOD, GPIO_Pin_4,
	GPIOD, GPIO_Pin_5,
	GPIOD, GPIO_Pin_6, 1);

  while (1)
  {
    //Step the vertical and vertical stepper 100 steps forward (90 degrees)
    //Stepper_Step(verticalStepper, 100);
	//Stepper_Step(horizontalStepper, 100);
	Stepper_StepTogether(horizontalStepper, verticalStepper, 100, 50);
	
	//Step the vertical and vertical stepper 200 steps backwards (-180 degrees)
	//Stepper_Step(verticalStepper, -200);
	//Stepper_Step(horizontalStepper, -200);
	Stepper_StepTogether(horizontalStepper, verticalStepper, -200, -100);
	
    //Step Back to original Positions
	//Stepper_Step(verticalStepper, 100);
	//Stepper_Step(horizontalStepper, 100);
	Stepper_StepTogether(horizontalStepper, verticalStepper, 100, 50);
	//Stepper_Enable(horizontalStepper);
	//Stepper_Enable(verticalStepper);
	//Stepper_DoubleStep(horizontalStepper, verticalStepper, 100);
    
	//Disable the steppers to conserve power.  They will stop resisting motion and can be
	//freely turned by external forces.  The steppers will be automatically enabled again
	//The next time a function that moves them is called (such as Stepper_Reset or Stepper_Step)
	Stepper_Disable(verticalStepper);
	Stepper_Disable(horizontalStepper);
	
    //Wait some time before repeating
	Delay(0xFFFFFF);
	Beep(5, GPIOD, GPIO_Pin_15);
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void Beep(int beeps, GPIO_TypeDef* beepBlock, uint16_t beepPin)
{
	while(beeps>0)
	{
		GPIO_SetBits(beepBlock, beepPin);
		Delay(0x5FFFFF);
		GPIO_ResetBits(beepBlock, beepPin);
		Delay(0x5FFFFF);
		beeps--;
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
