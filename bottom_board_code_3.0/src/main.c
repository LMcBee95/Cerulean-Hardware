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
	initialize_servo_timer();
	init_USART1(LASER_BAUD);  		//initializes USART1 baud rate

	init_USART2(LASER_BAUD);		// initialize USART2 baud rate
	init_UART5(BOTTOM_MOTOR_BAUD);	// initialize UART5 baud rate
	init_USART6(TOP_BOTTOM_BAUD); 	// initialize USART6 baud rate
	
	//initializes the pwm for the dc motors on the claw
	initialize_claw1_timer(100000 , 1);
	initialize_claw2_timer(100000 , 1);
	
	//initializes the two stepper motor objects
	initialize_stepper_objects();
	initialize_stepper_pins();
	

	initialize_led_timers(LED_PWM_FREQUENCY, 1);  //timer used to control the large leds on the cameras
	initialize_timer3(100000, 1);  //timer used for the pwm in mulitple different toolss
	initialize_timer5();  //initializes a timer that will be triggered once a ms
	
	init_muxes();  //initializes the camera muxes
	
	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);

	Delay(0xFFF); //Delays to give the read/write pin time to initialize
	
	init_LEDS();  //initializes the leds that are used to light up the cameras
	 
	 

	while (1)
	{  

		sendPackets();
		
		if(sendUpTrigger)
		{
			sendDataUp();
			sendUpTrigger = 0;
		}
		
		Delay(0xfffff);
		
		
		//servo integration test code
		//moves the two servos from 0 degrees to 100 degrees
		
		/*Delay(0xfffff);
		setServo1Angle(0);
		setServo2Angle(0);
		
		Delay(0xfffff);
		setServo1Angle(100);
		setServo2Angle(100);*/
		
		
		
		//LED integratin test code
		//turns the leds to 2/5 full power
		
		/*cameraLedPwm(100, 100, 100, 100, 100);*/
		
		
		//Voltage detection code
		//turns on the green led if a voltage is on VSEN1, the yellow led if VSEN2, and the red led if VSEN3
		
		/*if(ADC3ConvertedValue[VSEN1] < ADC_TO_VOLTS * ON_VOLTAGE)
			GPIO_SetBits(GPIOD, GPIO_Pin_11); 
		else
			GPIO_ResetBits(GPIOD, GPIO_Pin_11);
		
		if(ADC3ConvertedValue[VSEN2] < ADC_TO_VOLTS * ON_VOLTAGE)
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
		else
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		
		if(ADC3ConvertedValue[VSEN3] < ADC_TO_VOLTS * ON_VOLTAGE)
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
		else
			GPIO_ResetBits(GPIOD, GPIO_Pin_13);*/
			
		
		//Bilge pump
		//turns the bilge pump on
		
		/*bilgePumpPwm(1); */
		
		
		//turn foot motor
		//spins the motor in one direction, waits a little bit, and then turns the motor in the other direction

		/*turnFootPwm(150, 0);
		
		Delay(0xffffff);
		
		turnFootPwm(0, 150);
		
		Delay(0xffffff)*/
		
	return(0);
	}
}
