#ifndef __ADC_H
#define __ADC_H

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

extern uint16_t adc1[];
extern uint16_t adc2[];
extern uint16_t adc3[];

//functions to initiate the different ADC's
void startAdc1(uint8_t size);
void startAdc2(void);
void startAdc3(uint8_t size);


void addAdc(ADC_TypeDef* ADCx, GPIO_TypeDef* bank, uint16_t pinNum);


void bankToClock(GPIO_TypeDef* bank);



#endif