#ifndef BOTTOM_BOARD_FUNCTIONS_H_
#define BOTTOM_BOARD_FUNCTIONS_H_

/***************** LIBRARIES *****************/

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

/***************** CONSTANTS *****************/

/***  Serial Communication ***/
#define PACKET_SIZE 				16
#define SENT_PACKET_SIZE			12
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

#define POLL_MOTOR_TIME_OUT			3 

/*** Laser Measurement Tool ***/

#define LASER_BAUD 					115200
#define LASER_USART					USART1

/***  Direct Memory Access ***/

#define NUM_DMA_2_CONVERSIONS 		8

uint16_t ADC1ConvertedValue[NUM_DMA_2_CONVERSIONS];

/*** Discovery Board Debugging Led Pins ***/

#define GREEN_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_12);
#define GREEN_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
#define ORANGE_LED_ON				GPIO_SetBits(GPIOD, GPIO_Pin_13);
#define ORANGE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
#define RED_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_14);
#define RED_LED_OFF					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
#define BLUE_LED_ON					GPIO_SetBits(GPIOD, GPIO_Pin_15);
#define BLUE_LED_OFF				GPIO_ResetBits(GPIOD, GPIO_Pin_15);

/*** Servo 1 Init ***/

#define SERVO_1_TIMER_CLOCK			RCC_APB2Periph_TIM9
#define SERVO_TIMER_PIN_AF			GPIO_AF_TIM9
#define SERVO_TIMER					TIM9
#define SERVO_1_CLOCK_BANK			RCC_AHB1Periph_GPIOE
#define SERVO_BANK   				GPIOE
#define SERVO_1_PIN					GPIO_Pin_5
#define SERVO_1_PIN_SOURCE			GPIO_PinSource5
#define SERVO_1_CCR					TIM9->CCR1

/*** Servo 2 Init ***/

#define	SERVO_2_PIN 				GPIO_Pin_6
#define SERVO_2_PIN_SOURCE			GPIO_PinSource6
#define SERVO_2_CCR					TIM9->CCR2

#define LED_PIN1					GPIO_Pin_3
#define LED_PIN2					GPIO_Pin_2
#define LED_PIN3					GPIO_Pin_1

/*** Setting Servo Angle ***/

#define SERVO_PERIOD				26250 * 2  //this users clock 2 for timer 9; clock 2 is 2 times faster than clock 1
#define MAXSERVO 					2.1
#define MINSERVO 					0.8
#define MAXSERVOANGLE 				135.0

/*** Camera Leds ***/

#define LED_PWM_FREQUENCY			525000

#define LED_1_2_3_TIMER				TIM2
#define LED_1_2_3_AF				GPIO_AF_TIM2
#define LED_1_2_3_BANK				GPIOA

/*** Turning Foot and Bilge Pump ***/

#define TURN_FOOT_BANK_CLOCK		RCC_AHB1Periph_GPIOA
#define BILGE_PUMP_BANK_CLOCK		RCC_AHB1Periph_GPIOB
#define TURN_FOOT_BANK				GPIOA
#define BILGE_PUMP_BANK				GPIOB	

#define TURN_FOOT_PIN1				GPIO_Pin_6
#define TURN_FOOT_PIN2				GPIO_Pin_7
#define BILGE_PUMP_PIN1 			GPIO_Pin_0
#define BILGE_PUMP_PIN2 			GPIO_Pin_1

#define TURN_FOOT_SOURCE_PIN1		GPIO_PinSource6
#define TURN_FOOT_SOURCE_PIN2		GPIO_PinSource7
#define BILGE_PUMP_SOURCE_PIN1 		GPIO_PinSource0
#define BILGE_PUMP_SOURCE_PIN2 		GPIO_PinSource1

/*** Stepper Motor Init***/

#define STEPPER_TIMER_CLOCK			RCC_APB1Periph_TIM12
#define STEPPER_BANK_CLOCK			RCC_AHB1Periph_GPIOB
#define STEPPER_BANK				GPIOB	
#define STEPPER_TIMER				TIM12

#define STEPPER_PIN1				GPIO_Pin_14
#define STEPPER_PIN2				GPIO_Pin_15

#define STEPPER_SOURCE_PIN1			GPIO_PinSource14
#define STEPPER_SOURCE_PIN2			GPIO_PinSource15

/*** RGB Led Init ***/

#define RGB_TIMER_CLOCK				RCC_APB1Periph_TIM4
#define RGB_BANK_CLOCK				RCC_AHB1Periph_GPIOD	
#define RGB_BANK					GPIOD			
#define RGB_TIMER 					TIM4

#define RED_LED_PIN					GPIO_Pin_14
#define GREEN_LED_PIN				GPIO_Pin_13
#define BLUE_LED_PIN				GPIO_Pin_12

#define RED_LED_SOURCE_PIN			GPIO_PinSource14
#define GREEN_LED_SOURCE_PIN		GPIO_PinSource13
#define BLUE_LED_SOURCE_PIN			GPIO_PinSource12

#define RGB_AF						GPIO_AF_TIM4

/*** General PWM Information ***/

#define GENERAL_PWM_FREQUENCY		100000
#define GENERAL_PWM_PRESCALER		1
#define GENERAL_PWM_PERIOD			((84000000 * GENERAL_PWM_PRESCALER) / GENERAL_PWM_FREQUENCY)	


/***************** FUNCTION DECLARATIONS *****************/

void bilgePumpPwm(uint8_t dutyCycle1, uint8_t dutyCycle2, uint32_t period);

void cameraLedPwm(uint8_t dutyCycle,uint8_t dutyCycle2, uint8_t dutyCycle3, uint32_t period);

uint8_t checksum(uint8_t* packet, uint8_t size);

void convertTBtoBB(uint8_t* top);

void Delay(__IO uint32_t nCount);

void pollMotor(uint8_t address);

void resetMotor(uint8_t address);

void RGBLedPwm(uint8_t dutyCycleRed, uint8_t dutyCycleGreen, uint8_t dutyCycleBlue);

void sendPackets(void);

void setServo1Angle(uint8_t angle);

void setServo2Angle(uint8_t angle);

void stepperPwm(uint8_t dutyCycle1, uint8_t dutyCycle2);

void turnFootdPwm(uint8_t dutyCycle1, uint8_t dutyCycle2);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

/********* INITITALIZATION FUNCTIONS *********/

void init_DMA(uint16_t *storage_array, uint16_t size_of_array);

void init_IRQ(void);

void init_LEDS(void);

int32_t init_RGB_led_timers(uint32_t frequency, uint16_t preScaler);

void initialize_servo_timer(void);

int32_t initialize_stepper_timer(uint32_t frequency, uint16_t preScaler);

int32_t initialize_timer3(uint32_t frequency, uint16_t preScaler);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_USART6(uint32_t baudrate);

int32_t initialize_led_timers(uint32_t frequency, uint16_t preScaler);

#endif
