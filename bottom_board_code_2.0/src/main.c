

/*									       Information about this code
 *				
 *	This code currently takes a packet from the top board, converts it to packets the motor controllers can use, and
 *	then sends the packets out to the motors. The code will automatically poll a motor every twenty packets
 *	to check for any faults.
 *	
 *	The read/write enabler pin for the RS485 is D1. For USART6, the Rx is B7 and the Tx is B6. For USART2, 
 *	the Rx is A3 and the Tx is A2.	
 *
 *  The important part of the code is at the bottom, within main(). Everything before that is just initializing
 *  variables and creating functions.
 *
 */

#include "Bottom_Board_Functions.h"

int main(void) {

	init_IRQ();
	init_LEDS();
	
	init_USART6(BOTTOM_MOTOR_BAUD); 	// initialize USART6 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
	
	GPIO_SetBits(GPIOD, GPIO_Pin_12);


	//GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);	//Turns the read/write pin for rs485 to write mode
	Delay(0xFFF); //Delays to give the read/write pin time to initialize
	  
	while (1)
	{  
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		
		Delay(0x3fffff);
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		
		Delay(0x3fffff);
	}
}