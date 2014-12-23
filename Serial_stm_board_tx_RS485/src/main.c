#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
GPIO_InitTypeDef  GPIO_InitStructure;


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, data);
}

uint8_t crc8(uint8_t *packet)
{ 

  uint8_t crc = 0;
  *packet++;
  for(uint8_t len = 1; len < 5; len++) {
    uint8_t inbyte = *packet++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0xD5;
      inbyte >>= 1;
    }
  }
  return crc;
}

/* This funcion initializes the USART1 peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
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
	USART_InitStruct.USART_BaudRate = baudrate;				// the baud rate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
}

/* This function is used to transmit a string of characters via 
 * the USART specified in USARTx.
 * 
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 * 
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 * 
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */

 uint32_t delayTime= 0x0000FFFF;  //A delay to reduce the noise from sending serial too quickly.
 void send_packet(uint8_t command)
{
	uint8_t sendPacket[] = {0x12, 1, command, 4, 4, 5,0x13};
	
	USART_puts(USART1, sendPacket[0]);
	
    // PD12 to be toggled 
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
		
	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[1]);
	
    // PD12 to be toggled 
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
		
	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[2]);

    // PD12 to be toggled 
    GPIO_SetBits(GPIOD, GPIO_Pin_13);

	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[3]);

	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[4]);

	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[5]);

	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	USART_puts(USART1, sendPacket[6]);

	//Adds a delay to smooth out serial
	Delay(delayTime);
	
	Delay(0x00FFFFFF);
	//Delay(0x00FFFFFF);

    GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	GPIO_SetBits(GPIOD, GPIO_Pin_1);
}

int main(void) 
{

/* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 


  init_USART1(9600); // initialize USART1 baud rate

  USART_Cmd(USART1, ENABLE);
  GPIO_SetBits(GPIOD, GPIO_Pin_1);
  
  uint8_t sendPacket[] = {0x12, 1, 1, 4, 4, 5,0x13};
  sendPacket[5] = crc8(sendPacket);

  Delay(0x3FFFFF);

  uint8_t i = 1;
  uint8_t counter = 0;
  uint8_t storage[7];
  while (1){  
	
    send_packet(3);
	
	Delay(0x03FFFF); //delay 1 ms
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_1);
	
	while(!USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)){
		while(counter < 8)
		{
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) //if data is received store it in the array storage
			{
				storage[counter] = USART_ReceiveData(USART1);	//Reads in the data from the buffer into an array
				counter++;	//Increments the counter
			}
			
			if(counter > 7) //A packet has been received
			{
				if(crc8(storage) == storage[6])
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_15);	//Turns on led to indicate that serial is receiving data
				}				
			}
		}
		Delay(0x03FFFF);
		GPIO_SetBits(GPIOD, GPIO_Pin_1);
		counter = 0;
		
	}
  }
 }

