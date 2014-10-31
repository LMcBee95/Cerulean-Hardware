/*
 * usart.c
 */
#include "usart.h"



void Usart3Put(uint8_t ch)
{
	//put char to the buffer
	BufferPut(&U3Tx, ch);
	//enable Transmit Data Register empty interrupt
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}
uint8_t Usart3Get(void){
	uint8_t ch;
	//check if buffer is empty
	while (BufferIsEmpty(U3Rx) ==SUCCESS);
	BufferGet(&U3Rx, &ch);
	return ch;
}


//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
void BufferInit(__IO FIFO_TypeDef *buffer)
{
buffer->count = 0;//0 bytes in buffer
buffer->in = 0;//index points to start
buffer->out = 0;//index points to start
}

ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch)
{
if(buffer->count==USARTBUFFSIZE)
	return ERROR;//buffer full
buffer->buff[buffer->in++]=ch;
buffer->count++;
if(buffer->in==USARTBUFFSIZE)
	buffer->in=0;//start from beginning
return SUCCESS;
}

ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch)
{
if(buffer->count==0)
	return ERROR;//buffer empty
*ch=buffer->buff[buffer->out++];
buffer->count--;
if(buffer->out==USARTBUFFSIZE)
	buffer->out=0;//start from beginning
return SUCCESS;
}
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer)
{
	if(buffer.count==0)
		return SUCCESS;//buffer full
	return ERROR;
}
