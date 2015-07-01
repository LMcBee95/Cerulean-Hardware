

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "led.h"
#include "gpio.h"
#include "servo.h"



void Delay(__IO uint32_t nCount);

int main(void)
{

	
led myLeds;


	
gpio greenLed(GPIOD, GPIO_Pin_12);
gpio orangeLed(GPIOD, GPIO_Pin_13);
gpio redLed(GPIOD, GPIO_Pin_14);
gpio blueLed(GPIOD, GPIO_Pin_15); 
	
	servo myServo(GPIOA, GPIO_Pin_0, TIM2, 1);

  while (1)
  {
     
    myServo.setAngle(0);
	
	greenLed.on();
	  
	
	Delay(0x2FFFFFF);
    
	myServo.setAngle(170);
	//setServo1Angle(120);
	
	greenLed.off();
	
	/* Insert delay */
	Delay(0x2FFFFFF);
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


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
