

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "led.h"
#include "gpio.h"
#include "servo.h"
#include "serial.h"


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

uint8_t stuff[PACKETSIZE] = {1, 2, 3, 4};

serial mySerial(GPIOD, GPIO_Pin_5, GPIOD, GPIO_Pin_6, USART2, 9600);

  while (1)
  {
	green.on();

	mySerial.write(stuff, PACKETSIZE);
	
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
