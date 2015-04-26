

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
#include "stepper.h"

int main(void) {

	init_DMA_ADC1(ADC1ConvertedValue, NUM_DMA_ADC1_CONVERSIONS); //the function does not actually use the array address yet
	//init_DMA_ADC3(ADC3ConvertedValue, NUM_DMA_ADC3_CONVERSIONS);  //the function does not actually use the array address yet
	init_IRQ();
	init_RGB_led_timers(100000, 1);
	
	initialize_servo_timer();
	init_USART1(LASER_BAUD);  //initializes USART1 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
	init_UART5(BOTTOM_MOTOR_BAUD);	// initialize USART2 baud rate
	init_USART6(TOP_BOTTOM_BAUD); 	// initialize USART6 baud rate
	
	initialize_led_timers(LED_PWM_FREQUENCY, 1);
	initialize_timer3(100000, 1);
	initialize_stepper_timer(100000, 1);
	
<<<<<<< HEAD
	init_LEDS();
	initialize_claw1_timer(100000, 1);
	initialize_claw2_timer(100000, 1);
=======
	//init_LEDS();
	initialize_claw1_timer(100000, 1);
	initialize_claw2_timer(100000, 1);
>>>>>>> origin/master

	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);

	Delay(0xFFF); //Delays to give the read/write pin time to initialize

	RGBLedPwm(125, 125, 125);
	
	
	 int i; 
	 int j;
	 int k;
	  
	while (1)
<<<<<<< HEAD
	{  
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		clawPwm(255, 255);
		RGBLedPwm(255, 255, 255);
		GPIO_SetBits(GPIOD, GPIO_Pin_10);
		
		Delay(0xFFFFFF);
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);
		RGBLedPwm(10, 10, 10);
		clawPwm(0, 0);
		
		Delay(0xFFFFFF);
=======
	{  	
		/*	LED TEST CODE:  Connect a potentiometter to the claw current pin and you can then varry the led brightness  based on the reading
		
			int brightness = ADC1ConvertedValue[0] >> 6;  //reads an adc value from CLAWCURRENT PIN
		
			cameraLedPwm(brightness, brightness, brightness, brightness, brightness); 
		
		*/
		
		/*     	SERVO MOTOR AND CURRENT SENSOR TEST CODE: servo1_curr and servo2_current are the currents for the servos. 
			
		
		
			setServo1Angle(0);
			setServo2Angle(0);
		
		
			int servo1_curr = ADC1ConvertedValue[2] >> 4;
			int servo2_curr = ADC1ConvertedValue[3] >> 4;
		
			RGBLedPwm(255, servo1_curr, servo2_curr);
		
			Delay(0xfffff);
			
			setServo1Angle(100);
			setServo2Angle(100);
		
		
			servo1_curr = ADC1ConvertedValue[2] >> 4;
			servo2_curr = ADC1ConvertedValue[3] >> 4;
		
			RGBLedPwm(255, servo1_curr, servo2_curr);
			
		
			Delay(0xfffff);
		
		*/
		
		
		/*	TEST CODE FOR ADC 3
			
		
			
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			i = ADC1ConvertedValue[0] ;
			//j = ADC3ConvertedValue[0] ;
			k = ADC1ConvertedValue[2] ;
		
			
			if(i > 1000)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				
				GPIO_SetBits(GPIOD, GPIO_Pin_10);
			}
			
			if(j > 1000)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_11);
			}
			
			Delay(0xffff);
			
			
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				GPIO_ResetBits(GPIOD, GPIO_Pin_11);
				GPIO_ResetBits(GPIOD, GPIO_Pin_10);
		
			Delay(0xffff);
		*/	
>>>>>>> origin/master

	}
}