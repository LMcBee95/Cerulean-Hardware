

#include "stm32f4xx_conf.h"

/* C++ libraries */
#include "led.h"
#include "gpio.h"
#include "servo.h"
#include "pwm.h"
#include "adc.h"
#include "serial.h"
#include "usartDma.h"

gpio test(GPIOA, GPIO_Pin_14);
gpio another(GPIOA, GPIO_Pin_13);

int main(void)
{
	
	

  while (1)
  {
	test.on();
	another.off();
    
  }
}



