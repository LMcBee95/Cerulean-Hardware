   #include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
GPIO_InitTypeDef  GPIO_InitStructure;

uint8_t counter = 0;	//Counter used to increment the data received from the top board into an array
uint8_t storage[16];	//Array used to store the data received from the top board
uint8_t motor[8][7];	//Initializes a multidimensional array to store all of the motor commands


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/*
 * This function waits until it can write serial data and then 
 * transmits over the specified data through the designated USART port
 */

void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, data);;
		Delay(0x0000FFFF);//Added delay to reduce noise. Don't know why we need it, because the while loop should reduce the noise.
}


/* This function initializes USART1 peripheral
 * 
 * Arguments: baud rate --> the baud rate at which the USART is 
 * 						   supposed to operate
 */
void init_USART1(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	USART_Cmd(USART1, ENABLE);	//Enables USART1
}

void init_USART2(uint32_t baudrate){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//Initialized A2 as Tx and A3 as Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	// TX = PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE); //Enable USART2

}

uint8_t checksum(uint8_t* packet, uint8_t start_index, uint8_t size) {
	uint8_t crc = 0;
	*packet += start_index;
	for (uint8_t i = 0; i < size; i++) {
		uint8_t inbyte = *packet++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0xD5;
			inbyte >>= 1;
		}
	}
	return crc;
}

//Reads through the motor commands and sends them to the motors
void sendPackets(void){
	for(uint8_t i = 0; i < 8; i++)  //Cycles through all of the packets
	{
		for(uint8_t j = 0; j < 7; j++) //Cycles through all of the information in the packets
		{
			USART_puts(USART1, motor[i][j]);
		}
	}
	
}

//Converts the motor commands sent from the top board to packets that the motor controllers can read
void convertTBtoBB(uint8_t* top) {
	//Reads throught the motor values from the top packet
	for (int i = 0; i < 8; i++) {
		// 0 for reverse, 1 for forward.
		uint8_t direction = (top[i+ 1] < 128);
		// Make positive, bit shift to get values between 0 and 254, values will only be even.
		uint8_t magnitude = (top[i + 1] & 127) << 1;
		// Motor controller cannot accept 18.
		if (magnitude == 18)
			magnitude = 17;
		
		//Stores the correct values into the packets to be sent to the motors
		motor[i][0] = 0x12;
		motor[i][1] = i + 1;
		motor[i][2] = 1;
		
		motor[0][1] = 2;
		
		motor[i][3] = direction;
		motor[i][4] = magnitude;
		motor[i][5] = checksum(motor[i], 1, 4);
		motor[i][6] = 0x13;
	}
}


int main(void) {

/* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 


  init_USART1(9600); 	// initialize USART1 baud rate
  init_USART2(9600);	// initialize USART2 baud rate
  
  GPIO_ResetBits(GPIOD, GPIO_Pin_1);

  Delay(0x3FFFFF);

  /*
   *	USART1, rx: B7  tx: B6, used to communicate with the motors
   *	USART2, rx: A3  tx: A2, used to communicate with the top board
   */
  
  
  GPIO_SetBits(GPIOD, GPIO_Pin_1);	//Turns the read/write pin for rs485 to write mode
  Delay(0x0000FFFF); 
  
  while (1){  

	
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){
	  
		GPIO_SetBits(GPIOD, GPIO_Pin_12);	//Turns on led to indicate that serial is working
		
		while(counter < 16) //Loops until it reads the entire packet
		{
			if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
			{
				storage[counter] = USART_ReceiveData(USART2);	//Reads in the data from the buffer into an array
				counter++;	//Increments through the storrage variable
			}
		}
		
		if(counter > 15) //When we read in the entire top packet
		{
			if(1 || checksum(storage, 2, 13) == storage[14]) //If the checksum is correct
			{
				convertTBtoBB(storage);
				sendPackets(); //Sends out the motor commands
				//Send packet back to the main compunter
			}	
			else
			{
				//Send back that there was an error?
			}
			counter = 0;	
	    }
		
		Delay(0x0000FF);
	  
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);	//Turns off the led  
    }

  }
}
