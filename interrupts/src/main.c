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

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

GPIO_InitTypeDef  GPIO_InitStructure;

#define PACKET_SIZE 16
#define TOP_BOTTOM_BAUD 19200
#define BOTTOM_MOTOR_BAUD 57600


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/* This function initializes USART6 peripheral
 * 
 * Arguments: baud rate --> the baud rate at which the USART is 
 * 						   supposed to operate
 */
void init_USART6(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	 /*Enable the read write pins*/
	 
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART6 initialization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* enable APB2 peripheral clock for USART6 
	 * note that only USART6 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART6, PC6 for R and PC7 for TX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART6 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baud rate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pull up resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART6 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART6 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				  // the baud rate is set to the value we passed into this function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		  // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		  // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART6, &USART_InitStruct);					  // again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	/* Here the USART1 receive interrupt is enabled
	* and the interrupt controller is configured
	* to jump to the USART1_IRQHandler() function
	* if the USART1 receive interrupt occurs
	*/
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	USART_Cmd(USART6, ENABLE);	//Enables USART6
}


//Same set up as USART6 except for the USART2 pins
void init_USART2(uint32_t baudrate){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);

	//Initialized A2 as Tx and D6 as Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Initializes the D6 pin
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE); //Enable USART2

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt
}

int main(void) {

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configures the leds and the read/write pin on D1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	init_USART6(BOTTOM_MOTOR_BAUD); 	// initialize USART6 baud rate
	init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate

	Delay(0xFFFFF); //Delays to give the read/write pin time to initialize

	GPIO_SetBits(GPIOD, GPIO_Pin_15);

	while (1){
		//* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_14);

		/* Insert delay */
		Delay(0x3FFFFF);

		GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

		/* Insert delay */
		Delay(0xFFFFFF);
	}
}

void USART6_IRQHandler(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	return;
}

void USART2_IRQHandler(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
}