

#include "stm32f4xx_conf.h"

/* C++ libraries */
#include "led.h"
#include "gpio.h"
#include "servo.h"
#include "pwm.h"
#include "adc.h"
#include "serial.h"
#include "usartDma.h"

#include "interrupts.h"



 void Delay(__IO uint32_t nCount)
{
	while(nCount--)
	{
	}
}

gpio green(GPIOD, GPIO_Pin_12);  
gpio orange(GPIOD, GPIO_Pin_13);
gpio red(GPIOD, GPIO_Pin_14);
gpio blue(GPIOD, GPIO_Pin_15);

uint8_t packet[1] = {1};

int main(void)
{
	
	serial four(GPIOA, GPIO_Pin_1, GPIOA, GPIO_Pin_0, UART4, 9600);	
	
	usartDma DMAfour(UART4);
	
	
	//DMAfour.initTx(1, DMA1_Stream4, DMA_Channel_4);
	
	DMAfour.initRx(1);
	
	

  while (1)
  {
	packet[0] = DMAfour.get(0);
	  
	  
	  four.write(packet, 1);
	  red.toggle();
	  
	  Delay(0xffffff);
    
  }
}



