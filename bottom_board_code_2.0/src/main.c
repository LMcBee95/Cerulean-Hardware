

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
#include "Stepper.h"

int main(void) {

	init_DMA(ADC1ConvertedValue, NUM_DMA_2_CONVERSIONS);
	init_IRQ();
	init_LEDS();
	
	int16_t period = initialize_servo_timer();
	init_USART1(LASER_BAUD);  //initializes USART1 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
	init_USART6(BOTTOM_MOTOR_BAUD); 	// initialize USART6 baud rate
	
	int32_t timer = initialize_pwm_timers(525000, 1);
	
	

	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);

	Delay(0xFFF); //Delays to give the read/write pin time to initialize
	  
	while (1)
	{  
		
		setServo1Angle(130);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		//anologWrite(TIM3->CCR4, 255, timer);
		
		Delay(0x1ffffff);
		
		setServo1Angle(0);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		//anologWrite(TIM3->CCR4, 125, timer;
		
		Delay(0x1ffffff);
		
		
	}
}