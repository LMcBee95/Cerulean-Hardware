#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include "Bottom_Board_Functions.h"


uint8_t checksum(uint8_t* packet, uint8_t size) {
	uint8_t crc = 0;
	
	for (uint8_t i = 1; i < size + 1; i++) {
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

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

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
		USART_puts(USART6, poll[i]);
}

void sendPackets(void){
	for(uint8_t i = 0; i < 8; i++)  //Cycles through all of the packets
	{
		for(uint8_t j = 0; j < 7; j++) //Cycles through all of the information in the packets
		{
			USART_puts(USART6, motor[i][j]);
		}
	}
	
}

uint8_t readSlavePacket(void)
{
	uint32_t timer = 0;  //Variable to make the code break out of the next while loop if no data comes
	uint8_t received = 0;  //Stores initial variable received
	Delay(0x3FFF);		//Wait for the read write pin to turn low
	GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
	GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN); 
	
	while(timer < 0x4FFFF)
	{
		if(USART_GetFlagStatus(USART6, USART_FLAG_RXNE))
		{
			received = USART_ReceiveData(USART6);
			
			if(received == 0x12)
			{
				uint8_t counter = 1;
				pollReceived[0] = 0x12;
				timer = 0;  //resets the timer after it recieves the state byte
				while(counter < 7 && timer < 0x4FFFF)  //Cycles through the data until all 7 byte are read
				{
					while(USART_GetFlagStatus(USART6, USART_FLAG_RXNE)) //if data is received store it in the array pollReceived
					{
						pollReceived[counter] = USART_ReceiveData(USART6);	//Reads in the data from the buffer into an array
						counter++;	//Increments the counter
					}
					timer++; 
				}
				if(checksum(pollReceived, 4) == pollReceived[5] && pollReceived[6] == 0x13)  //If the check sum and end byte are correct
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_15);	//Turns on led to indicate that serial is receiving data
					
					GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //Sets the read/write pin to write mode
					GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN); 
					return(1); //Reading the packet was successful
				}
				else	
				{
					GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //Sets the read/write pin to write mode
					GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN); 
					return(0);  //Did not read the packet successfully
				}
			}
		}
		timer++;	//Increases timer until data is received or timer is greater than 0x4FFFF
	}
	GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //Sets the read/write pin to write mode
	GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN); 
	return(0); //No data was received
}

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
		USART_puts(USART6, reset[i]);
}


//UNDER PROGRESS
void USART2_IRQHandler(void) {
    //Check if interrupt was because data is received
    if (USART_GetITStatus(USART2, USART_IT_RXNE)) 
	{	
		received = USART_ReceiveData(USART2);
		
		if(received == 0x12)
		{
			storage[counter] = received;
			counter = 1;
		}
		else if(counter > 0 && received != 0x12)
		{
			storage[counter] = received;
			counter++;
			
			if(counter == PACKET_SIZE  && (checksum(storage, PACKET_SIZE - 3) == storage[PACKET_SIZE - 2]) && (storage[PACKET_SIZE - 1] == 0x13))
			{
				convertTBtoBB(storage);  //Converts the data from the top board into motor controller commands that we can use
				sendPackets();	//Sends the motor controller commands produced by the convert function
				counter = 0; //Reset the counter
				pollCounter++;
				
				if(pollCounter > 20)
				{
					pollMotor(pollAddress);
					
					if(!readSlavePacket())
					{
					
					}
					
					pollAddress++;
					if(pollAddress == 9)
					{
						pollAddress = 1;
					}
					pollCounter = 0;  //Resets the poll counter
					
				}
			}
		}
		else
		{
			counter = 0;
		}
	}
}

void USART6_IRQHandler(void) {
    //Check if interrupt was because data is received
    if (USART_GetITStatus(USART6, USART_IT_RXNE)) {
        //Do your stuff here
        
        //Clear interrupt flag
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    }
}


void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

