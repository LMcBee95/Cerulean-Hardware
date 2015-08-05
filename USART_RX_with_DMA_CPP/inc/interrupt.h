#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?


uint8_t Buffer[3];
 

void DMA1_Stream2_IRQHandler(void);


#endif