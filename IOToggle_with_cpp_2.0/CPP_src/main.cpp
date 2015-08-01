

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

	#define PACKETSIZE 4


	uint8_t stuff[PACKETSIZE] = {5, 2, 3, 6};

	serial mySerial(GPIOC, GPIO_Pin_11, GPIOC, GPIO_Pin_10, UART4, 9600);
	
	usartDma myDma(UART4);
	myDma.initTx(3, DMA1_Stream4, DMA_Channel_4);
	
	myDma.insert(0, 0);
	myDma.insert(1, 1);
	myDma.insert(2, 2);

  while (1)
  {
	green.on();

	myDma.write();
	
	Delay(0xcfffff);
	
	//mySerial.write();
	green.off();
	
	Delay(0xcfffff);
    
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
