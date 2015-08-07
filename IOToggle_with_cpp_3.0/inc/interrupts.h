#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?

#include "DMA.h"

//the struct used to initialize the DMA channel

DMA_InitTypeDef  DMA1_stream0_structure;  //UART5
DMA_InitTypeDef  DMA1_stream1_structure;  //USART3
DMA_InitTypeDef  DMA1_stream2_structure;  //UART4
DMA_InitTypeDef  DMA1_stream3_structure;
DMA_InitTypeDef  DMA1_stream4_structure;
DMA_InitTypeDef  DMA1_stream5_structure;  //USART2
DMA_InitTypeDef  DMA1_stream6_structure;
DMA_InitTypeDef  DMA1_stream7_structure;
DMA_InitTypeDef  DMA1_stream8_structure;

DMA_InitTypeDef  DMA2_stream1_structure; //USART6
DMA_InitTypeDef  DMA2_stream5_structure; //USART1


/*DMA1 interrupts */

/* USART5 Rx */
void DMA1_Stream0_IRQHandler(void);
/* USART3 Rx */
void DMA1_Stream1_IRQHandler(void);
/* UART4 Rx;  I2C_2 Rx;  I2C 3 Rx */
void DMA1_Stream2_IRQHandler(void);
/*USART3 Rx */
void DMA1_Stream3_IRQHandler(void);
/* UART4 Tx;  USART3 Tx */
void DMA1_Stream4_IRQHandler(void);
/* USART2 Rx */
void DMA1_Stream5_IRQHandler(void);
/* USART2 Tx */
void DMA1_Stream6_IRQHandler(void);
/* UART5 Tx */
void DMA1_Stream7_IRQHandler(void);

/* DMA2 interrupts */

void DMA2_Stream0_IRQHandler(void);
/* USART6_Rx */
void DMA2_Stream1_IRQHandler(void);
/* USART6 Rx; USART1 Rx */
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
/* USART1 Rx */
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);


#endif