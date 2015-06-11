/*									       Information about this code!
 *				
 *	This code currently takes a packet from the top board, converts it to packets the motor controllers can use, and
 *	sends the packets out to the motors. The code also takes in controlls for a variety of tools that the user can 
 *	control. The ROV then sends information back up to the base station about sensors and statuses.
 *
 *  	All of the functions are located in Bottom_Board_Functions.c with descriptions of the functions in it's respective
 *	header file. The majority of the controls and reading in data occur in void USART6_IRQHandler(void), and 
 *	TIM5_IRQHandler(void), and the while(1) looop in the main function.
 */

#include "Bottom_Board_Functions.h"  //library that contains all of the functions that control the ROV
#include "stepper.h"  //library used to control and keep track of stepper motors

int main(void) {
	setSteppersDebugByte(0xF1);

	init_DMA_ADC1(ADC1ConvertedValue, NUM_DMA_ADC1_CONVERSIONS); //the function does not actually use the array address yet
	init_DMA_ADC3(ADC3ConvertedValue, NUM_DMA_ADC3_CONVERSIONS);  //the function does not actually use the array address yet
	init_IRQ();  //initiates all of the interrupts that are used for serial communication and timing
	
	init_RGB_led_timers(100000, 1);
	
	initialize_servo_timer();
	init_USART1(LASER_BAUD);  		//initializes USART1 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
	init_UART5(BOTTOM_MOTOR_BAUD);	// initialize USART2 baud rate
	init_USART6(TOP_BOTTOM_BAUD); 	// initialize USART6 baud rate
	
	//initializes the pwm for the dc motors on the claw
	initialize_claw1_timer(100000 , 1);
	initialize_claw2_timer(100000 , 1);
	
	//initializes the two stepper motor objects
	initialize_stepper_objects();
	initialize_stepper_pins();
	

	initialize_led_timers(LED_PWM_FREQUENCY, 1);  //timer used to control the large leds on the cammeras
	initialize_timer3(100000, 1);  //timmer used for the pwm in mulitple different toolss
	initialize_timer5();  //initializes a timer that will be triggered once a ms
	
	init_muxes();  //initializes the camera muxes
	
	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);

	Delay(0xFFF); //Delays to give the read/write pin time to initialize
	
	 init_LEDS();  //initializes the leds that are used to light up the cammeras
	 

	while (1)
	{  
		sendPackets();
		Delay(0xffffff);
		
		//GPIO_SetBits(GPIOD, GPIO_Pin_10);
		
		Delay(0xfffff);
		
		//GPIO_ResetBits(GPIOD, GPIO_Pin_10);
	}
	
	return(0);
}
