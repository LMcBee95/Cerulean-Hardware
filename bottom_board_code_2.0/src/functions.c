
#define PACKET_SIZE 11
#define TOP_BOTTOM_BAUD 19200
#define BOTTOM_MOTOR_BAUD 28800

#define GREED_LED GPIO_Pin_12
#define ORANGE_LED GPIO_Pin_13
#define RED_LED GPIO_Pin_14
#define BLUE_LED GPIO_Pin_15

#define READ_WRITE_ENABLER GPIO_Pin_1

#define PACKETS_SENT_BEFORE_POLLED 20





//Creates a check sum 
//You pass in the name of the array, and how many bytes you want the checksum to read
uint8_t checksum(uint8_t* packet, uint8_t packetSize) {
	uint8_t crc = 0;
	
	for (uint8_t i = 1; i < packetSize + 1; i++) {
		uint8_t inbyte = packet[i];
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

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void convertTBtoBB(uint8_t* top)
{
	//Reads through the motor values from the received top packet
	for (int i = 0; i < 8; i++) 
	{
		
		// 0 for reverse, 1 for forward
		uint8_t direction = (top[i+ 1] < 128);
		
		// Removes the first byte that gave the direction, and 
		//Bit shifts the rest of the number to multiply the value by two to get the values between 0 and 254
		uint8_t magnitude = (top[i + 1] & 127) << 1;
		
		// Motor controller cannot accept 18 so we round 18 down to 17
		if (magnitude == 18)
			magnitude = 17;
		
		//Stores the correct values into the packets to be sent to the motors
		motor[i][1] = i + 1;
		motor[i][2] = 1;
		motor[i][3] = direction;
		motor[i][4] = magnitude;
		motor[i][5] = checksum(motor[i], 4);
		motor[i][6] = 0x13;
		motor[i][0] = 0x12;  //This is out of place because it gives me errors if I set the start byte value first
	}
}

/*
 *	This function handles receiving the top packet and then sending out the commands to the motor  
 *	controllers. It takes no arguments. The function will keep cycling until it receives a 0x12. Then 
 *	function stores the top board information into an array, produces motor control commands from the 
 *	top packet,and then sends the commands to all of the motors. The function checks before it sends 
 * 	the command to make sure the check sum is correct and that the last byte is a 0x13. If this is 
 * 	true then the function returns a one and if it isn't, then the function returns a 0.
 */

uint8_t handleTopPacket(void)
{
	
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
	{
		uint8_t received = USART_ReceiveData(USART2);
		
		if(0x12 == received)
		{
			uint8_t timer = 0;	//Timer used to stop the function from waiting for data if there is an error
			GPIO_SetBits(GPIOD, GREED_LED);	//Turns on the green led 
			uint8_t counter = 1;	//Counter used to count how many bytes we have read in from the top borad
			while(counter < PACKET_SIZE && timer < 0xFFFF)  //Waits until all 16 bytes are read in. If no data comes then the function will break 
												   //out after a short period of time
			{
				if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) //if data is received store it in the array storage
				{
					received = USART_ReceiveData(USART2);
					if(received != 0x12)
					{
						storage[counter] = received;	//Reads in the data from the buffer into an array
						counter++;	//Increments the counter
					}
					else
					{
						return(0);
					}
					
					
				}
				GPIO_SetBits(GPIOD, ORANGE_LED);
				timer++;
			}	
			if(1 || (checksum(storage, PACKET_SIZE - 3) == storage[PACKET_SIZE - 2]) && (storage[PACKET_SIZE - 1] == 0x13))  //Checks the check sum and the end byte
			{	
				GPIO_ResetBits(GPIOD, ORANGE_LED);
				convertTBtoBB(storage);  //Converts the data from the top board into motor controller commands that we can use
				sendPackets();	//Sends the motor controller commands produced by the convert function
				return(1); //Reading the packet was successful!
			}
			else	
			{
				USART_puts(USART1, (checksum(storage, PACKET_SIZE - 3)));
				USART_puts(USART1,storage[PACKET_SIZE - 2]);
				return(0);  //Returns 0 if the check sum or end byte were incorrect
			}
			
		}
		else
			return(0);  //Makes the function recursive until we get a response from the top board
	}
	else
		return(0);
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
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
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
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baud rate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pull up resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				  // the baud rate is set to the value we passed into this function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		  // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		  // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					  // again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	USART_Cmd(USART1, ENABLE);	//Enables USART1
}

//Same set up as USART1 except for the USART2 pins
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

/*
 *This function sends a packet to poll a motor.
 *It takes the address of the motor being polled as its argument
 */
void pollMotor(uint8_t address)
{
	//Stores each variable into the array
	poll[1] = address;  //The address of the motor that we are going to poll
	poll[2] = 3;
	poll[3] = 0;
	poll[4] = 0;
	poll[5] = checksum(poll,4);
	poll[6] = 0x13;
	poll[0] = 0x12;  //This packet is out of place because it gives errors if this value is assigned first

	//Sends the packet to poll the motor
	for(uint8_t i = 0; i < 7; i++)
		USART_puts(USART1, poll[i]);
}

/*
 *	This function reads the data that the motors send back after we poll them.
 *	It reads in data just like the handleTopPacket function. It will return a 
 *	1 if it received data from the motor, correct check sum and end byte, or 
 * 	returns a 0 if it did not read the packet successfully.
 */

uint8_t readSlavePacket(void)
{
	uint32_t timer = 0;  //Variable to make the code break out of the next while loop if no data comes
	uint8_t received = 0;  //Stores initial variable received
	Delay(0x3FFF);		//Wait for the read write pin to turn low
	GPIO_ResetBits(GPIOD, READ_WRITE_ENABLER);
	
	
	while(timer < 0x4FFFF)
	{
		if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
		{
			received = USART_ReceiveData(USART1);
			
			if(received == 0x12)
			{
				uint8_t counter = 1;
				pollReceived[0] = 0x12;
				timer = 0;  //resets the timer after it recieves the state byte
				while(counter < 7 && timer < 0x4FFFF)  //Cycles through the data until all 7 byte are read
				{
					while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) //if data is received store it in the array pollReceived
					{
						pollReceived[counter] = USART_ReceiveData(USART1);	//Reads in the data from the buffer into an array
						counter++;	//Increments the counter
					}
					timer++; 
				}
				if(checksum(pollReceived, 4) == pollReceived[5] && pollReceived[6] == 0x13)  //If the check sum and end byte are correct
				{
					GPIO_SetBits(GPIOD, BLUE_LED);	//Turns on blue led to indicate that serial is receiving data
					
					GPIO_SetBits(GPIOD, READ_WRITE_ENABLER);  //Sets the read/write pin to write mode
					return(1); //Reading the packet was successful
				}
				else	
				{
					GPIO_SetBits(GPIOD, READ_WRITE_ENABLER);  //Sets the read/write pin to write mode
					return(0);  //Did not read the packet successfully
				}
			}
		}
		timer++;	//Increases timer until data is received or timer is greater than 0x4FFFF
	}
	GPIO_SetBits(GPIOD, READ_WRITE_ENABLER);  //Sets the read/write pin to write mode
	return(0); //No data was received
}

/*
 *This function sends a packet to rest a motor if there is an error
 *It takes the address of the motor being reset as its argument
 */
void resetMotor(uint8_t address)
{
	//Stores each variable into the array
	reset[1] = address;  //The address of the motor that we are going to reset
	reset[2] = 4;
	reset[3] = 0;
	reset[4] = 0;
	reset[5] = checksum(reset, 4);
	reset[6] = 0x13;
	reset[0] = 0x12;
	
	//Sends the packet to reset the motor
	for(uint8_t i = 0; i < 7; i++)
		USART_puts(USART1, reset[i]);
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

/*
 * This function waits until it can write serial data and then 
 * transmits the specified data through the designated USART port
 */
 void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}