/*
 * usart.h
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f4xx.h"

//#include "functions.h"

#define USARTBUFFSIZE	19

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

volatile FIFO_TypeDef U3Rx, U3Tx;
void Usart3Init(void);
void Usart3Put(uint8_t ch);
uint8_t Usart3Get(void);


#endif /* USART_H_ */
