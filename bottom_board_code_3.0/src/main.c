

/*									       Information about this code!
 *				
 *	This code currently takes a packet from the top board, converts it to packets the motor controllers can use, and
 *	then sends the packets out to the motors. The code will automatically poll a motor every twenty packets
 *	to check for any faults.
 *	
 *	The read/write enabler pin for the RS485 is D1. For USART6, the Rx is B7 and the Tx is B6. For USART2, 
 *	the Rx is A3 and the Tx is A2.	
 *
 *  The important part of the code is at the bottom, within main(). Everything before that is just initializing
 *  variables and creating functions
 *
 */

#include "Bottom_Board_Functions.h"
#include "stepper.h"

int main(void) {
	setSteppersDebugByte(0xF1);

	init_DMA_ADC1(ADC1ConvertedValue, NUM_DMA_ADC1_CONVERSIONS); //the function does not actually use the array address yet
	init_DMA_ADC3(ADC3ConvertedValue, NUM_DMA_ADC3_CONVERSIONS);  //the function does not actually use the array address yet
	init_IRQ();
	
	init_RGB_led_timers(100000, 1);
	
	initialize_servo_timer();
	init_USART1(LASER_BAUD);  //initializes USART1 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
	init_UART5(BOTTOM_MOTOR_BAUD);	// initialize USART2 baud rate
	init_USART6(TOP_BOTTOM_BAUD); 	// initialize USART6 baud rate
	
	initialize_claw1_timer(100000 , 1);
	initialize_claw2_timer(100000 , 1);
	
	initialize_stepper_objects();
	initialize_stepper_pins();
	

	//initialize_stepper_timer(100000, 1);
	initialize_led_timers(LED_PWM_FREQUENCY, 1);
	initialize_timer3(100000, 1);
	initialize_timer5();
	

	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);

	Delay(0xFFF); //Delays to give the read/write pin time to initialize
	
	RGBLedPwm(0, 0, 0);
	 init_LEDS();


	while (1)
	{  
		
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_13 );
		GPIO_ResetBits(GPIOD, GPIO_Pin_12 );
		GPIO_ResetBits(GPIOD, GPIO_Pin_11 );
		GPIO_ResetBits(GPIOD, GPIO_Pin_10 );
		
		clawPwm(175, 0);
		//turnFootdPwm(175, 0);
		setServo1Angle(120);
		setServo2Angle(120);
		bilgePumpPwm(100);
		

		Delay(0xffffff);

	}
	
	return(0);
}
