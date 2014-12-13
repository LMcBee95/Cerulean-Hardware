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
//#include "stm32f4xx_usart.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
//NVIC_InitTypeDef NVIC_InitStructure;
/* Private define ------------------------------------------------------------*/
typedef uint8_t BottomPacket[7];
typedef uint8_t TopPacket[16];
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t index = 0;
uint8_t packet_in_progress = 0;
int GPIO[] = { GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15 };
int leds[] = { 0, 0, 0, 0 };
/* Private function prototypes -----------------------------------------------*/
uint8_t checksum(uint8_t* packet);
void ConfigurePin(GPIO_TypeDef* GPIOx, uint32_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, GPIOOType_TypeDef type, GPIOPuPd_TypeDef pupd);
void Delay(__IO uint32_t nCount);
void Initialize();
void LEDStartupRoutine();
void newBottomPacket(uint8_t* bp, uint8_t address, uint8_t cmd, uint8_t arg1, uint8_t arg2);
void SetLED(uint8_t led, uint8_t value);
void SetPinMode(uint8_t pin, uint8_t mode, GPIO_TypeDef* block);
void ToggleLED(uint8_t led);
void USART_puts(USART_TypeDef* USARTx, volatile uint8_t *s);
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
	
	// Rx
	uint8_t bottomindex, topindex;
	TopPacket TopPacketRx;
	BottomPacket BottomPacketRx;

	// Tx
	TopPacket TopPacketTx;
	BottomPacket BottomPacketTx;

	newBottomPacket(BottomPacketTx, 0, 2, 2, 3);

	while (1) {
		Delay(0x0FFFFF);
		// Recieve Data
		// From top board
		SetLED(0, 1);
		ToggleLED(1);
		
		USART_puts(USART1, BottomPacketTx);
	}
}

void newBottomPacket(uint8_t* bp, uint8_t address, uint8_t cmd, uint8_t arg1, uint8_t arg2) {

	bp[0] = 0x12;
	bp[1] = address;
	bp[2] = cmd;
	bp[3] = arg1;
	bp[4] = arg2;
	bp[5] = checksum(bp);
	bp[6] = 0x13;
}

uint8_t checksum(uint8_t* packet) {
	uint8_t crc = 0;
	*packet++;
	for (uint8_t len = 1; len < 5; len++) {
		uint8_t inbyte = *packet++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0xD5;
			inbyte >>= 1;
		}
	}
	return crc;
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
/* Set up all pins for use.*/
void Initialize() {
	/* Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(USER_BUTTON_GPIO_CLK, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	ConfigurePin(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, \
		GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

	// USART1 on PB6 (TX) & PB7 (RX)
	ConfigurePin(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF, GPIO_Speed_100MHz, \
		GPIO_OType_PP, GPIO_PuPd_UP);

	// USART2 on PA2 (TX) & PA3 (RX)
	ConfigurePin(GPIOA, GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AF, GPIO_Speed_100MHz, \
		GPIO_OType_PP, GPIO_PuPd_UP);

	// Configure Button in output pushpull mode
	ConfigurePin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, GPIO_Mode_IN, \
		GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

	// Initialize USART
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART1, &USART_InitStructure);
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE); // Enable USART
	USART_Cmd(USART2, ENABLE); // Enable USART
}
/* This function is used to transmit a string of characters via
* the USART specified in USARTx.
*
* It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
* 						   (volatile) char *s is the string you want to send
*
* Note: The string has to be passed to the function as a pointer because
* 		 the compiler doesn't know the 'string' data type. In standard
* 		 C a string is just an array of characters
*
* Note 2: At the moment it takes a volatile char because the received_string variable
* 		   declared as volatile char --> otherwise the compiler will spit out warnings
* */
void USART_puts(USART_TypeDef* USARTx, volatile uint8_t *s){
	uint8_t sent = 0;
	while (sent != 0x13) {
		// wait until data register is empty
		while (!(USARTx->SR & 0x00000040));
		USART_SendData(USARTx, *s);
		sent = *s;
		s++;
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
/*8-step light sequence*/
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
