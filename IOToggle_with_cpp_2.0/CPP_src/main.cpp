

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

/* C++ libraries */
#include "led.h"
#include "gpio.h"
#include "servo.h"
#include "pwm.h"
#include "adc.h"
#include "serial.h"
#include "usartDma.h"

void Delay(__IO uint32_t nCount);

void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

int main(void)
{
	
	gpio green(GPIOD, GPIO_Pin_12);
	gpio orange(GPIOD, GPIO_Pin_13);
	gpio red(GPIOD, GPIO_Pin_14);
	gpio blue(GPIOD, GPIO_Pin_15);
	
	serial info(GPIOA, GPIO_Pin_1, GPIOA, GPIO_Pin_0, UART4, 9600);
	
	
	uint8_t message[1] = {1};
	
	
	adc readings;
	readings.addAdcPin(ADC2, GPIOC, GPIO_Pin_0);
	readings.addAdcPin(ADC2, GPIOC, GPIO_Pin_1);
	readings.startAdc2(DMA2_Stream2 ,DMA_Channel_1);
	

  while (1)
  {
	message[0] = readings.get(2, 0) >> 4;
	info.write(message, sizeof(message)); 
	  
	message[0] = readings.get(2, 1) >> 4;
	info.write(message, sizeof(message)); 
	  
	green.on();

	
	Delay(0xcffff);
	

	green.off();
	
	Delay(0xcffff);
    
  }
}


void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
