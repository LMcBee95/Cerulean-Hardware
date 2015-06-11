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
#include "stepper.h"

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

#define READ_LASER					(storage[10]) & 0x10

/***  Direct Memory Access ***/

#define NUM_DMA_ADC1_CONVERSIONS 		5  //the size of the ADC1 array
#define NUM_DMA_ADC3_CONVERSIONS 		4  //the size of the ADC3 array

//Array locations of adc 1 readings
#define CLAW_CURRENT					0 
#define VOLT_48_CURRENT				1
#define SERVO2CURRENT				2
#define SERVO1CURRENT				3
#define TURNCURRENT					4

//Array locations of adc 3 readings
#define VSEN3						0
#define VSEN2						1
#define VSEN1						2
#define FUSEDETECTM					3	


uint16_t ADC1ConvertedValue[NUM_DMA_ADC1_CONVERSIONS];  //array to store the ADC1 values
uint16_t ADC3ConvertedValue[NUM_DMA_ADC3_CONVERSIONS];  //array to store the ADC3 values


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

/*** Setting Servo Angle ***/

#define SERVO_PERIOD				26250 * 2  //this users clock 2 for timer 9; clock 2 is 2 times faster than clock 1
#define MAXSERVO 					2.1
#define MINSERVO 					0.8
#define MAXSERVOANGLE 				135.0

/*** Camera Leds ***/

#define LED_PWM_FREQUENCY		525000
#define LED_PWM_PERIOD			((84000000 * 1) / LED_PWM_FREQUENCY)

#define LED_PIN1					GPIO_Pin_3
#define LED_PIN2					GPIO_Pin_2
#define LED_PIN3					GPIO_Pin_1
#define LED_PIN4					GPIO_Pin_9
#define LED_PIN5					GPIO_Pin_8

#define LED_SOURCE_PIN1				GPIO_PinSource1
#define LED_SOURCE_PIN2				GPIO_PinSource2
#define LED_SOURCE_PIN3				GPIO_PinSource3
#define LED_SOURCE_PIN4				GPIO_PinSource9
#define LED_SOURCE_PIN5				GPIO_PinSource8


#define LED_1_2_3_TIMER				TIM2
#define LED_1_2_3_AF				GPIO_AF_TIM2
#define LED_1_2_3_BANK				GPIOA

#define LED_4_TIMER					TIM14
#define LED_4_AF					GPIO_AF_TIM14
#define LED_4_BANK					GPIOF
#define LED_4_PIN					

#define LED_5_TIMER					TIM13
#define LED_5_AF					GPIO_AF_TIM13
#define LED_5_BANK					GPIOF

#define LED1_VALUE					storage[12]
#define LED2_VALUE					storage[13]
#define LED3_VALUE					storage[14]
#define LED4_VALUE					storage[15]
#define LED5_VALUE					storage[16]

/*** Turning Foot and Bilge Pump ***/

#define TURN_FOOT_BANK_CLOCK		RCC_AHB1Periph_GPIOA
#define BILGE_PUMP_BANK_CLOCK		RCC_AHB1Periph_GPIOB
#define TURN_FOOT_BANK				GPIOA
#define BILGE_PUMP_BANK				GPIOB	

#define TURN_FOOT_PIN1				GPIO_Pin_6
#define TURN_FOOT_PIN2				GPIO_Pin_7
#define BILGE_PUMP_PIN1 			GPIO_Pin_0


#define TURN_FOOT_SOURCE_PIN1		GPIO_PinSource6
#define TURN_FOOT_SOURCE_PIN2		GPIO_PinSource7
#define BILGE_PUMP_SOURCE_PIN1 		GPIO_PinSource0
#define BILGE_PUMP_SOURCE_PIN2 		GPIO_PinSource1

#define MAX_BILGE_PUMP_VALUE		155
#define BILGE_PUMP_VALUE			(storage[10] & (1 << 3))

#define FOOT_TURNER_VALUE 			storage[9]

/*** Voltage Sesnors ***/

#define READ_VOLTAGES				(storage[10] & 0x08)

/*** Stepper Motor Init***/

#define STEPPER_TIMER_CLOCK			RCC_APB1Periph_TIM12
#define STEPPER_BANK_CLOCK			RCC_AHB1Periph_GPIOB
#define STEPPER_BANK				GPIOB	
#define STEPPER_TIMER				TIM12

#define STEPPER_PIN1				GPIO_Pin_14
#define STEPPER_PIN2				GPIO_Pin_15

#define STEPPER_SOURCE_PIN1			GPIO_PinSource14
#define STEPPER_SOURCE_PIN2			GPIO_PinSource15

/*** Stepper Motor Pins ***/
#define STEPPER_STEP_BANK						GPIOB
#define STEPPER_DIR_ENABLE_BANK					GPIOC

/*#define STEPPER_HORIZONTAL_DIR_BANK         GPIOC
#define STEPPER_HORIZONTAL_ENABLE_BANK        GPIOC
#define STEPPER_HORIZONTAL_STEP_BANK          GPIOB

#define STEPPER_VERTICAL_DIR_BANK             GPIOC
#define STEPPER_VERTICAL_ENABLE_BANK          GPIOC
#define STEPPER_VERTICAL_STEP_BANK            GPIOB*/

#define STEPPER_HORIZONTAL_DIR_PIN        GPIO_Pin_15
#define STEPPER_HORIZONTAL_ENABLE_PIN     GPIO_Pin_13
#define STEPPER_HORIZONTAL_STEP_PIN       GPIO_Pin_15

#define STEPPER_VERTICAL_DIR_PIN          GPIO_Pin_14
#define STEPPER_VERTICAL_ENABLE_PIN       GPIO_Pin_13
#define STEPPER_VERTICAL_STEP_PIN         GPIO_Pin_14

#define STEPPER_VERTICAL_POLARITY    1  //If the stepper turns the wrong way, just
#define STEPPER_HORIZONTAL_POLARITY  1  //change the polarity from 1 to 0 or from 0 to 1.

#define STEPPER_UP_BYTE 4               //Write 4 bytes to the upgoing packet, starting at this one
#define STEPPER_DOWN_BYTE 11            //Read this byte from the packet from the surface to control steppers

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
#define CAMERA_LED_PWM_PERIOD		((84000000 * GENERAL_PWM_PRESCALER) / LED_PWM_FREQUENCY)




/***************** FUNCTION DECLARATIONS *****************/

void bilgePumpPwm(uint8_t bilgePumpOn);

void cameraLedPwm(uint8_t led1DutyCycle, uint8_t led2DutyCycle, uint8_t led3DutyCycle, uint8_t led4DutyCycle, uint8_t led5DutyCycle);

uint8_t checksum(uint8_t* packet, uint8_t size);

void clawPwm(uint8_t PWM_IN1, uint8_t PWM_IN2);

void convertTBtoBB(uint8_t* top);

void Delay(__IO uint32_t nCount); 	

void pollMotor(uint8_t address);

void resetMotor(uint8_t address);

void RGBLedPwm(uint8_t dutyCycleRed, uint8_t dutyCycleGreen, uint8_t dutyCycleBlue);

void sendPackets(void);

void setServo1Angle(uint8_t angle);

void setServo2Angle(uint8_t angle);

//Use storage packet to set stepper values and write the new stepper positions to the dataUpPacket
void setSteppers(void);

void setSteppersDebugByte(uint8_t byte);

void stepperPwm(uint8_t dutyCycle1, uint8_t dutyCycle2);

void TIM1_UP_TIM10_IRQHandler(void);

void turnFootPwm(uint8_t PWM_IN1, uint8_t PWM_IN2);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

/********* INITITALIZATION FUNCTIONS *********/

void initialize_claw1_timer(uint32_t frequency, uint16_t preScaler);

void initialize_claw2_timer(uint32_t frequency, uint16_t preScaler);

void init_DMA_ADC1(uint16_t *array, uint16_t size);

void init_DMA_ADC3(uint16_t *array, uint16_t size);

void init_IRQ(void);

void init_LEDS(void);

void init_RGB_led_timers(uint32_t frequency, uint16_t preScaler);

void init_muxes(void);

void initialize_servo_timer(void);

void initialize_stepper_pins();

void initialize_stepper_timer(uint32_t frequency, uint16_t preScaler);

void initialize_stepper_objects(void);

void initialize_timer3(uint32_t frequency, uint16_t preScaler);

void initialize_timer5(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_UART5(uint32_t baudrate);

void init_USART6(uint32_t baudrate);

void initialize_led_timers(uint32_t frequency, uint16_t preScaler);

#endif
