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
#include "stm32f4xx_usart.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
NVIC_InitTypeDef NVIC_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t index = 0, pressed = 0;
int GPIO[] = { GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15 };
int leds[] = { 0, 0, 0, 0 };
/* Private function prototypes -----------------------------------------------*/
void ConfigurePin(GPIO_TypeDef* GPIOx, uint32_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, GPIOOType_TypeDef type, GPIOPuPd_TypeDef pupd);
void Delay(__IO uint32_t nCount);
void Initialize();
void LEDStartupRoutine();
void SetLED(uint8_t led, uint8_t value);
void SetPinMode(uint8_t pin, uint8_t mode, GPIO_TypeDef* block);
void ToggleLED(uint8_t led);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {
	/*!< At this stage the microcontroller clock setting is already configured, 
		 this is done through SystemInit() function which is called from startup
		 file (startup_stm32f4xx.s) before to branch to application main.
		 To reconfigure the default setting of SystemInit() function, refer to
		 system_stm32f4xx.c file
	*/
	Initialize();

	LEDStartupRoutine();
	while (1) {
		Delay(0x3FFFFF);
		index++;
		/*if (index == 0) {
			index = 255;
		}
		else if (index == 255) {
			index = 0;
		}*/
		USART_SendData(USART1, 'm');
		uint16_t rec = USART_ReceiveData(USART1);
		rec %= 4;
		SetLED(0, (rec == 0));
		SetLED(1, (rec == 1));
		SetLED(2, (rec == 2));
		SetLED(3, (rec == 3));
	}
}

void ConfigurePin(GPIO_TypeDef* GPIOx, uint32_t pin, GPIOMode_TypeDef mode,\
		GPIOSpeed_TypeDef speed, GPIOOType_TypeDef type, GPIOPuPd_TypeDef pupd) {
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_OType = type;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_InitStructure.GPIO_PuPd = pupd;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/* If value is 0, turns led off. Otherwise turn led on.*/
void SetLED(uint8_t led, uint8_t value) {
	if (value != 0) {
		GPIO_SetBits(GPIOD, GPIO[led]);
		leds[led] = 1;
	}
	else {
		GPIO_ResetBits(GPIOD, GPIO[led]);
		leds[led] = 0;
	}
}
/* If led is on, turn off. If led is off, turn on.*/
void ToggleLED(uint8_t led) {
	if (leds[led]) {
		SetLED(led, 0);
	}
	else {
		SetLED(led, 1);
	}
}

void LEDStartupRoutine() {
	// Turn on in sequence
	SetLED(0, 1);
	Delay(0x3FFFFF);
	SetLED(1, 1);
	Delay(0x3FFFFF);
	SetLED(2, 1);
	Delay(0x3FFFFF);
	SetLED(3, 1);
	Delay(0x3FFFFF);
	// Turn off in sequence
	SetLED(0, 0);
	Delay(0x3FFFFF);
	SetLED(1, 0);
	Delay(0x3FFFFF);
	SetLED(2, 0);
	Delay(0x3FFFFF);
	SetLED(3, 0);
	Delay(0x3FFFFF);
}

void Initialize() {
	/* Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(USER_BUTTON_GPIO_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	ConfigurePin(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, \
		GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

	// USART on PB6 (TX) & PB7 (RX)
	ConfigurePin(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF, GPIO_Speed_100MHz, \
		GPIO_OType_PP, GPIO_PuPd_UP);

	// Configure Button in output pushpull mode
	ConfigurePin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, GPIO_Mode_IN, \
		GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

	// Initialize USART
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE); // Enable USART
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount) {
	while(nCount--){}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) { 
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
