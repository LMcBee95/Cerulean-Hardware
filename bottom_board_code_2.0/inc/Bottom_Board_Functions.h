#ifndef BOTTOM_BOARD_FUNCTIONS_H_
#define BOTTOM_BOARD_FUNCTIONS_H_

/********* LIBRARIES *********/

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <misc.h>			 

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

/********* CONSTANTS *********/

/***  Serial Communication ***/
#define PACKET_SIZE 				16
#define MOTOR_PACKET_SIZE			7
#define NUMBER_OF_MOTORS			8
#define TOP_BOTTOM_BAUD 			115200
#define BOTTOM_MOTOR_BAUD 			57600

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

#define START_BYTE					0x12
#define END_BYTE 					0x13

#define POLL_MOTOR_TIME_OUT			0 

/*** Laser Measurement Tool ***/

#define LASER_BAUD 					115200
#define LASER_USART					USART1

/***  Direct Memory Access ***/

#define NUM_DMA_2_CONVERSIONS 		8

/*** Led Pins ***/

#define GREEN_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_12);
#define GREEN_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
#define ORANGE_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_13);
#define ORANGE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
#define RED_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_14);
#define RED_LED_OFF					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
#define BLUE_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_15);
#define BLUE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_15);

/*** servo 1 init ***/

#define SERVO_1_TIMER_CLOCK			RCC_APB2Periph_TIM9
#define SERVO_1_TIMER_PIN_AF		GPIO_AF_TIM9
#define SERVO_1_TIMER				TIM9
#define SERVO_1_CLOCK_BANK			RCC_AHB1Periph_GPIOE
#define SERVO_1_BANK   				GPIOE
#define SERVO_1_PIN					GPIO_Pin_5
#define SERVO_1_PIN_SOURCE			GPIO_PinSource5
#define SERVO_1_CCR					TIM9->CCR1

/*** Setting Servo Angle ***/

#define SERVO_PERIOD				26250 * 2  //this users clock 2 for timer 9; clock 2 is 2 times faster than clock 1
#define MAXSERVO 					2.1
#define MINSERVO 					0.8
#define MAXSERVOANGLE 				135.0

#define NUM_DMA_CONVERSIONS			8

uint16_t ADC3ConvertedValue[NUM_DMA_CONVERSIONS]; //Array that stores all of the values for the DMA ADC
uint16_t ADC1ConvertedValue[NUM_DMA_CONVERSIONS]; //Array that stores all of the values for the DMA ADC

/********* FUNCTION DECLARATIONS *********/

void anologWrite(uint32_t channel, uint8_t dutyCycle, uint32_t period);

uint8_t checksum(uint8_t* packet, uint8_t size);

void convertTBtoBB(uint8_t* top);

void Delay(__IO uint32_t nCount);

uint8_t handleTopPacket(void);

void pollMotor(uint8_t address);

void resetMotor(uint8_t address);

void sendPackets(void);

void setServo1Angle(uint8_t angle);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

/********* INITITALIZATION FUNCTIONS *********/

void init_DMA(uint16_t *storage_array, uint16_t size_of_array);

void init_IRQ(void);

void init_LEDS(void);

uint16_t initialize_servo_timer(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_USART6(uint32_t baudrate);

int32_t initialize_pwm_timers(uint32_t frequency, uint16_t preScaler);

#endif
