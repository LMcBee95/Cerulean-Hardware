#ifndef BOTTOM_BOARD_FUNCTIONS_H_
#define BOTTOM_BOARD_FUNCTIONS_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder

#include "stm32f4xx_conf.h"
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_it.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_usart.h>

#include "Bottom_Board_Functions.h"

#define PACKET_SIZE 				16
#define MOTOR_PACKET_SIZE			7
#define TOP_BOTTOM_BAUD 			115200
#define BOTTOM_MOTOR_BAUD 			57600
#define LASER_BAUD 					115200
#define POLL_MOTOR_TIME_OUT			3 

#define NUM_DMA_CONVERSIONS 		7

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

#define GREEN_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_12);
#define GREEN_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
#define ORANGE_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_13);
#define ORANGE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
#define RED_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_14);
#define RED_LED_OFF					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
#define BLUE_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_15);
#define BLUE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_15);

uint16_t ADC3ConvertedValue[NUM_DMA_CONVERSIONS]; //Stores all of the values for the DMA ADC

uint8_t checksum(uint8_t* packet, uint8_t size);

void convertTBtoBB(uint8_t* top);

void Delay(__IO uint32_t nCount);

uint8_t handleTopPacket(void);

void pollMotor(uint8_t address);

uint8_t readSlavePacket(void);

void resetMotor(uint8_t address);

void sendPackets(void);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

//Initializations
void init_DMA(uint16_t *storage_array, uint16_t size_of_array);

void init_IRQ(void);

void init_LEDS(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_USART6(uint32_t baudrate);



#endif
