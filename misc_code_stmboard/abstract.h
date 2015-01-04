#ifndef ABSTRACT_H_
#define ABSTRACT_H_

#include "functions.h"


//start X5 code
//SENSORS, COMS, AND OTHER THINGS


//SPI1 - onboard accelerometer SPI1: PA7-MOSI, PA6-MISO, PA5-SCK, Enable x2: PE0, PE1, why PE3 is needed is unknown


//i2c1 - IMU  - PB8 - SCL , PB7 - SDA


//SPI2 - for solenoid selection - SPI2: PB15, PB10, PD3, PD2


//5x ADC - for power sensing - ADC1: PA1, PA4, PB0, PB1, PC5


//Debug Leds - PD12, PD13, PD14, PD15																//have initinalizations
#define LED_0_PIN					GPIO_Pin_12
#define LED_0_PORT					GPIOD
#define LED_0_CLK					RCC_AHB1Periph_GPIOD

#define LED_1_PIN					GPIO_Pin_13
#define LED_1_PORT					GPIOD
#define LED_1_CLK					RCC_AHB1Periph_GPIOD

#define LED_2_PIN					GPIO_Pin_14
#define LED_2_PORT					GPIOD
#define LED_2_CLK					RCC_AHB1Periph_GPIOD

#define LED_3_PIN					GPIO_Pin_15
#define LED_3_PORT					GPIOD
#define LED_3_CLK					RCC_AHB1Periph_GPIOD


//4x GPIO - for water detection  - Input floating: PD0, PC12, PC11, PC10							//have initinalizations
#define WATER_0_PIN					GPIO_Pin_0
#define WATER_0_PORT				GPIOD
#define WATER_0_CLK					RCC_AHB1Periph_GPIOD

#define WATER_1_PIN					GPIO_Pin_12
#define WATER_1_PORT				GPIOC
#define WATER_1_CLK					RCC_AHB1Periph_GPIOC

#define WATER_2_PIN					GPIO_Pin_11
#define WATER_2_PORT				GPIOC
#define WATER_2_CLK					RCC_AHB1Periph_GPIOC

#define WATER_3_PIN					GPIO_Pin_10
#define WATER_3_PORT				GPIOC
#define WATER_3_CLK					RCC_AHB1Periph_GPIOC


//2x GPIO - Turner selection - Output push pull: PE8, PE7											//have initinalizations
#define TURNER_SEL_0_PIN			GPIO_Pin_8
#define TURNER_SEL_0_PORT			GPIOE
#define TURNER_SEL_0_CLK			RCC_AHB1Periph_GPIOE

#define TURNER_SEL_1_PIN			GPIO_Pin_7
#define TURNER_SEL_1_PORT			GPIOE
#define TURNER_SEL_1_CLK			RCC_AHB1Periph_GPIOE


//MUX CONTROL - 2x GPIO - for camera selection PC11, PC12											//have initinalizations
#define MUX_0_PIN					GPIO_Pin_11
#define MUX_0_PORT					GPIOC
#define MUX_0_CLK					RCC_AHB1Periph_GPIOC

#define MUX_1_PIN					GPIO_Pin_12
#define MUX_1_PORT					GPIOC
#define MUX_1_CLK					RCC_AHB1Periph_GPIOC


//1x GPIO for fuse blown - Input floating:PE10														//have initinalizations
#define FUSE_DECT_PIN				GPIO_Pin_10
#define FUSE_DECT_PORT				GPIOE
#define FUSE_DECT_CLK				RCC_AHB1Periph_GPIOE


//PB5 for Turner  DIR 																				//have initinalizations
#define TURNER_DIR_PIN				GPIO_Pin_5
#define TURNER_DIR_PORT				GPIOB
#define TURNER_DIR_CLK				RCC_AHB1Periph_GPIOB


//1x PWM - Turner motors - TIM1: PE9, PE11, PE13, PE14 - May not need all!							//have initinalizations
//UNIVERSAL
#define TURNER_PWM_TIM_RCC			RCC_APB2Periph_TIM1
#define TURNER_PWM_TIM_AF			GPIO_AF_TIM1
#define TURNER_PWM_TIM				TIM1

//TURNER1
#define TURNER_PWM_1_PIN			GPIO_Pin_9
#define TURNER_PWM_1_PORT			GPIOE
#define TURNER_PWM_1_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_1_CH				1
#define TURNER_PWM_1_SOURCE			GPIO_PinSource9
//TURNER2
#define TURNER_PWM_2_PIN			GPIO_Pin_11
#define TURNER_PWM_2_PORT			GPIOE
#define TURNER_PWM_2_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_2_CH				2
#define TURNER_PWM_2_SOURCE			GPIO_PinSource11

//TURNER3
#define TURNER_PWM_3_PIN			GPIO_Pin_13
#define TURNER_PWM_3_PORT			GPIOE
#define TURNER_PWM_3_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_3_CH				3
#define TURNER_PWM_3_SOURCE			GPIO_PinSource13

//TURNER4
#define TURNER_PWM_4_PIN			GPIO_Pin_14
#define TURNER_PWM_4_PORT			GPIOE
#define TURNER_PWM_4_CLK			RCC_AHB1Periph_GPIOE
#define TURNER_PWM_4_CH				4
#define TURNER_PWM_4_SOURCE			GPIO_PinSource14


//Usart2: PD6-RX, PA2-TX											//have initinalizations
#define USART2_TIM_RCC				RCC_APB1Periph_USART2
#define USART2_TIM_AF 				GPIO_AF_USART2

#define USART2_RX_PIN				GPIO_Pin_6
#define USART2_RX_PORT				GPIOD
#define USART2_RX_CLK				RCC_AHB1Periph_GPIOD

#define USART2_TX_PIN				GPIO_Pin_2
#define USART2_TX_PORT				GPIOA
#define USART2_TX_CLK				RCC_AHB1Periph_GPIOA


//Usart6: PC7-RX, PC6-TX - Motor bus also needs PC8, PC9
#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_TIM_RCC				RCC_APB1Periph_USART2
#define USART6_TIM_AF 				GPIO_AF_USART2

#define USART6_RX_PIN				GPIO_Pin_7
#define USART6_RX_PORT				GPIOC
#define USART6_RX_CLK				RCC_AHB1Periph_GPIOC

#define USART6_TX_PIN				GPIO_Pin_6
#define USART6_TX_PORT				GPIOC
#define USART6_TX_CLK				RCC_AHB1Periph_GPIOC


//4x Extra GPIO: PD7, PE6, PE4, PE2																	//have initinalizations
#define GPIO_0_PIN					GPIO_Pin_7
#define GPIO_0_PORT					GPIOD
#define GPIO_0_CLK					RCC_AHB1Periph_GPIOD
  
#define GPIO_1_PIN					GPIO_Pin_6
#define GPIO_1_PORT					GPIOE
#define GPIO_1_CLK					RCC_AHB1Periph_GPIOE

#define GPIO_2_PIN					GPIO_Pin_4
#define GPIO_2_PORT					GPIOE
#define GPIO_2_CLK					RCC_AHB1Periph_GPIOE

#define GPIO_3_PIN					GPIO_Pin_2
#define GPIO_3_PORT					GPIOE
#define GPIO_3_CLK					RCC_AHB1Periph_GPIOE


//USART



#define USART3_TIM_RCC				RCC_APB1Periph_USART3
#define USART3_TIM_AF 				GPIO_AF_USART3
#define USART3_TIM					TIM14 //TIM14 because we dont use it anywhere else...this goes in the TIM input of PinInit to declare usart functionality


#define USART3_RX_PIN					GPIO_Pin_11
#define USART3_RX_PORT					GPIOB
#define USART3_RX_CLK					RCC_AHB1Periph_GPIOB


#define USART3_TX_PIN					GPIO_Pin_10
#define USART3_TX_PORT					GPIOB
#define USART3_TX_CLK					RCC_AHB1Periph_GPIOB


//gonna need to add a timer "identifier" and whater perihpial RCC this is on
//I2C 1
#define I2C_1_SDA_PIN					GPIO_Pin_7
#define I2C_1_SDA_PORT					GPIOB
//not sure if need clock here

#define I2C_1_SCL_PIN					GPIO_Pin_6
#define I2C_1_SCL_PORT					GPIOB
//and here

//I2C 3
#define I2C_3_SDA_PIN					GPIO_Pin_9
#define I2C_3_SDA_PORT					GPIOC
//clock?

#define I2C_3_SCL_PIN					GPIO_Pin_8
#define I2C_3_SCL_PORT					GPIOA
//clock?


//solenoid
//enable held low for operating - active low
//regclr held high for working, low for all zero        
//serial- in for datas
//clock high then low for clock inbetween each data bit
// PB15 - MOSI
// PB10 - Clk
// PD3  - LE
// PD2  - OE

#define SHIFT_MOSI_PIN						GPIO_Pin_15
#define SHIFT_MOSI_PORT						GPIOB
#define SHIFT_MOSI_CLK						RCC_AHB1Periph_GPIOB

#define SHIFT_CLK_PIN						GPIO_Pin_10
#define SHIFT_CLK_PORT					    GPIOB
#define SHIFT_CLK_CLK						RCC_AHB1Periph_GPIOB

#define SHIFT_LE_PIN						GPIO_Pin_3
#define SHIFT_LE_PORT					    GPIOD
#define SHIFT_LE_CLK						RCC_AHB1Periph_GPIOD

#define SHIFT_OE_PIN						GPIO_Pin_2
#define SHIFT_OE_PORT						GPIOD
#define SHIFT_OE_CLK						RCC_AHB1Periph_GPIOD


//UNIVERSAL
#define MOTOR1_TIM_RCC					RCC_APB1Periph_TIM4		
#define MOTOR1_TIM_AF					GPIO_AF_TIM4
#define MOTOR1_TIM				        TIM4		
	
//tim4_ch1
#define MOTOR1_PWM_PIN					GPIO_Pin_12
#define MOTOR1_PWM_PORT					GPIOD
#define MOTOR1_PWM_CLK					RCC_AHB1Periph_GPIOD
#define MOTOR1_PWM_CH					1

#define MOTOR1_DIR_PIN					GPIO_Pin_6
#define MOTOR1_DIR_PORT					GPIOE
#define MOTOR1_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim4_ch2
#define MOTOR2_PWM_PIN					GPIO_Pin_13
#define MOTOR2_PWM_PORT					GPIOD
#define MOTOR2_PWM_CLK					RCC_AHB1Periph_GPIOD
#define MOTOR2_PWM_CH					2

#define MOTOR2_DIR_PIN					GPIO_Pin_4
#define MOTOR2_DIR_PORT					GPIOE
#define MOTOR2_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim4_ch3
#define MOTOR3_PWM_PIN					GPIO_Pin_14
#define MOTOR3_PWM_PORT					GPIOD
#define MOTOR3_PWM_CLK					RCC_AHB1Periph_GPIOD
#define MOTOR3_PWM_CH					3

#define MOTOR3_DIR_PIN					GPIO_Pin_2
#define MOTOR3_DIR_PORT					GPIOE
#define MOTOR3_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim4_ch4
#define MOTOR4_PWM_PIN					GPIO_Pin_15
#define MOTOR4_PWM_PORT					GPIOD
#define MOTOR4_PWM_CLK					RCC_AHB1Periph_GPIOD
#define MOTOR4_PWM_CH					4

#define MOTOR4_DIR_PIN					GPIO_Pin_7
#define MOTOR4_DIR_PORT					GPIOD
#define MOTOR4_DIR_CLK					RCC_AHB1Periph_GPIOD



//PE6, PE4, PE2, PD7, PC13, PE5, PE12, PE15 - for direction
//pd12-15 and pe9, pe11, pe13, pe14 - for motor PWM


//UNIVERSAL
#define MOTOR2_TIM_RCC					RCC_APB1Periph_TIM3		
#define MOTOR2_TIM_AF					GPIO_AF_TIM3
#define MOTOR2_TIM						TIM3		
	

//tim3_ch1s
#define MOTOR5_PWM_PIN					GPIO_Pin_4
#define MOTOR5_PWM_PORT					GPIOB
#define MOTOR5_PWM_CLK					RCC_AHB1Periph_GPIOB
#define MOTOR5_PWM_CH					1

#define MOTOR5_DIR_PIN					GPIO_Pin_13
#define MOTOR5_DIR_PORT					GPIOC
#define MOTOR5_DIR_CLK					RCC_AHB1Periph_GPIOC

//tim3_ch2
#define MOTOR6_PWM_PIN					GPIO_Pin_5
#define MOTOR6_PWM_PORT					GPIOB
#define MOTOR6_PWM_CLK					RCC_AHB1Periph_GPIOB
#define MOTOR6_PWM_CH					2

#define MOTOR6_DIR_PIN					GPIO_Pin_5
#define MOTOR6_DIR_PORT					GPIOE
#define MOTOR6_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim3_ch3
#define MOTOR7_PWM_PIN					GPIO_Pin_0
#define MOTOR7_PWM_PORT					GPIOB
#define MOTOR7_PWM_CLK					RCC_AHB1Periph_GPIOB
#define MOTOR7_PWM_CH					3

#define MOTOR7_DIR_PIN					GPIO_Pin_12
#define MOTOR7_DIR_PORT					GPIOE
#define MOTOR7_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim1_ch4
#define MOTOR8_PWM_PIN					GPIO_Pin_1
#define MOTOR8_PWM_PORT					GPIOB
#define MOTOR8_PWM_CLK					RCC_AHB1Periph_GPIOB
#define MOTOR8_PWM_CH					4

#define MOTOR8_DIR_PIN					GPIO_Pin_15
#define MOTOR8_DIR_PORT					GPIOE
#define MOTOR8_DIR_CLK					RCC_AHB1Periph_GPIOE

/*
//tim1_ch1s
#define MOTOR5_PWM_PIN					GPIO_Pin_9
#define MOTOR5_PWM_PORT					GPIOE
#define MOTOR5_PWM_CLK					RCC_AHB1Periph_GPIOE
#define MOTOR5_PWM_CH					1

#define MOTOR5_DIR_PIN					GPIO_Pin_13
#define MOTOR5_DIR_PORT					GPIOC
#define MOTOR5_DIR_CLK					RCC_AHB1Periph_GPIOC

//tim1_ch2
#define MOTOR6_PWM_PIN					GPIO_Pin_11
#define MOTOR6_PWM_PORT					GPIOE
#define MOTOR6_PWM_CLK					RCC_AHB1Periph_GPIOE
#define MOTOR6_PWM_CH					2

#define MOTOR6_DIR_PIN					GPIO_Pin_5
#define MOTOR6_DIR_PORT					GPIOE
#define MOTOR6_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim1_ch3
#define MOTOR7_PWM_PIN					GPIO_Pin_13
#define MOTOR7_PWM_PORT					GPIOE
#define MOTOR7_PWM_CLK					RCC_AHB1Periph_GPIOE
#define MOTOR7_PWM_CH					3

#define MOTOR7_DIR_PIN					GPIO_Pin_12
#define MOTOR7_DIR_PORT					GPIOE
#define MOTOR7_DIR_CLK					RCC_AHB1Periph_GPIOE

//tim1_ch4
#define MOTOR8_PWM_PIN					GPIO_Pin_14
#define MOTOR8_PWM_PORT					GPIOE
#define MOTOR8_PWM_CLK					RCC_AHB1Periph_GPIOE
#define MOTOR8_PWM_CH					4

#define MOTOR8_DIR_PIN					GPIO_Pin_15
#define MOTOR8_DIR_PORT					GPIOE
#define MOTOR8_DIR_CLK					RCC_AHB1Periph_GPIOE
*/













#endif