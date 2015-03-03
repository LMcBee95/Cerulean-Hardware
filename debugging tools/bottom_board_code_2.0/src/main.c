/*									       Information about this code
 *				
 *	This code currently takes a packet from the top board, converts it to packets the motor controllers can use, and
 *	then sends the packets out to the motors. The code will automatically poll a motor every twenty packets
 *	to check for any faults.
 *	
 *	The read/write enabler pin for the RS485 is D1. For USART1, the Rx is B7 and the Tx is B6. For USART2, 
 *	the Rx is A3 and the Tx is A2.	
 *
 *  The important part of the code is at the bottom, within main(). Everything before that is just initializing
 *  variables and creating functions.
 *
 */


#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include "functions.h"
GPIO_InitTypeDef  GPIO_InitStructure;






uint8_t storage[PACKET_SIZE];	 //Array used to store the data received from the top board
uint8_t motor[8][7];	 //A multidimensional array to store all of the motor commands
uint8_t poll[7]; 		 //An array to store the packet that will poll the motors
uint8_t pollReceived[7]; //An array used to store the packet received from the motors after they are polled
uint8_t pollCounter = 0; //Keeps track of how many packets have been sent since we last polled a motor
uint8_t pollAddress = 1; //Stores the address of the motor that is going to be pulled next
uint8_t reset[7];		 //An array to send a reset command if one of the motors has a fault
 

int main(void) {


  init_pins();	//Initializes the leds and the read write enabler pins as outputs

  init_USART1(BOTTOM_MOTOR_BAUD); 	// initialize USART1 baud rate
  init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate

  GPIO_SetBits(GPIOD, READ_WRITE_ENABLER);	//Turns the read/write pin for rs485 to write mode
  Delay(0xFFF); //Delays to give the read/write pin time to initialize
  
  while (1){  

		//Reads the top packet, converts the top packet to motor packets, and then sends the motor packets to the motor controllers

		if(handleTopPacket())  //If the read top packet function was successful, go within the if statement
		{			
			
			GPIO_ResetBits(GPIOD, BLUE_LED);
			
			//Increments how many times the motor packets have been sent
			pollCounter++;
				
			//Waits for twenty packets to be sent to the motors before polling a motor.
			if(pollCounter > PACKETS_SENT_BEFORE_POLLED)
			{
						GPIO_SetBits(GPIOD, RED_LED);
				
				//Sends a packet to poll the motor at pollAddress
				pollMotor(pollAddress);	
				
				if(!readSlavePacket())  //If we cannot read the packet
				{
					//TODO
				}
				//Add the fault data information to the packet we are sending back to the battle station						
				
				//Increments through the addresses and goes back to address one after eight time
				pollAddress++;  //Changes which motor will be polled next
				if(pollAddress == 9)
					pollAddress = 1;
				pollCounter = 0;  //Resets the poll counter
			}
		}	
	  
		GPIO_ResetBits(GPIOD, GREED_LED);	//Turns off the led 
		GPIO_ResetBits(GPIOD, RED_LED);		
    }

  }