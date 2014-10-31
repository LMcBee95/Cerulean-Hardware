#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "stm32f4xx.h"
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

#include "abstract.h"

#include <stdio.h>

#define USARTBUFFSIZE	20

typedef struct{
	uint8_t in;
	uint8_t out;
	uint8_t count;
	uint8_t buff[USARTBUFFSIZE];
}FIFO_TypeDef;

void BufferInit(__IO FIFO_TypeDef *buffer);
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch);
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch);
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer);

extern volatile FIFO_TypeDef U2Rx, U2Tx;
extern volatile uint8_t flag2;
void Usart2Init(void);
void Usart2Put(uint8_t ch);
uint8_t Usart2Get(void);

extern volatile FIFO_TypeDef U6Rx, U6Tx;
extern volatile uint8_t flag6;
void Usart6Init(void);
void Usart6Put(uint8_t ch);
uint8_t Usart6Get(void);

/****************** ARRAYS AND THINGS! ******************/

/****************** ACTUAL FUNCTIONS AND STUFF ******************/


#include "functions.h"

//Usart6: PC7, PC6 - Motor bus also need PC8, PC9

//Output for 1, AF output for 2, input for else


/*************************** X5 Code ********************************************/
void PinOutput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK);

void PinInput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK);

void Mux_Init();

void TurnerSelectInit();

void TurnerDirInit();

void WaterInit();

void FuseInit();

void TurnerPWMInit();

void NVIC_Config(void);

void LedInit();

void Usart2Init();

void Usart6Init(); 

void MUXControl(volatile uint8_t M0, volatile uint8_t M1);

void delay(int i);

uint16_t GenChecksum(uint16_t Address, uint16_t Command);

void ResetMotorAddress();

void UpdateAddress(uint16_t OldAddress, uint16_t NewAddress);

void MotorControlRS485(uint16_t Address, uint16_t Speed);

void RequestStatus(uint16_t Address);

void ShiftInit();

void ShiftControl( volatile uint8_t manip, volatile uint8_t tool2, volatile uint8_t tool3, volatile uint8_t tool4);

void MotorSet(int Motor, int Dir, int Speed);

void MotorControl( volatile uint8_t M1, volatile uint8_t M2, volatile uint8_t M3, volatile uint8_t M4, volatile uint8_t M5, volatile uint8_t M6, volatile uint8_t M7, volatile uint8_t M8);

void Motor1Init();

void Motor2Init();

void Motor3Init();

void Motor4Init();

void Motor5Init();

void Motor6Init();

void Motor7Init();

void Motor8Init();

/****************** Usart & Buffer Functions ******************/
void Usart2Put(uint8_t ch);

uint8_t Usart2Get(void);

void Usart6Put(uint8_t ch);

uint8_t Usart6Get(void);

void BufferInit(__IO FIFO_TypeDef *buffer);

ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch);

ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch);

ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer);


/****************** Function that initializes all pin types (I think) ******************/


#endif