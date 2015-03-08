#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

uint8_t tempLaserData[10];  //Stores the decimal values of the length of the measurement.
uint16_t laserDataBuff[100];  //Stores the actual value of the length in mm from the target

volatile uint8_t laserSerialCounter = 0;  //Counter that determines where in the tempLaserData to store the next value
volatile uint8_t dataMeasurementCounter = 0;  //Counter that determines 
volatile uint8_t twoPreviousValue = 0;  //The value from two serial readings ago
volatile uint8_t previousValue = 0;    //The value from the last serial reading
volatile uint8_t currentValue = 0;     //The current value from the serial

void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

//initializes pin B6 as TX and B7 as RX
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
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;                         // the pins are configured as alternate function so the USART peripheral has access to them
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // this defines the IO speed and has nothing to do with the baud rate!
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                        // this defines the output type as push pull mode (as opposed to open drain)
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;                        // this activates the pull up resistors on the IO pins
        GPIO_Init(GPIOB, &GPIO_InitStruct);                                        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
        
        /* The RX and TX pins are now connected to their AF
         * so that the USART1 can take over control of the 
         * pins
         */
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
        
        /* Now the USART_InitStruct is used to define the 
         * properties of USART1 
         */
        USART_InitStruct.USART_BaudRate = baudrate;                                  // the baud rate is set to the value we passed into this function
        USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // we want the data frame size to be 8 bits (standard)
        USART_InitStruct.USART_StopBits = USART_StopBits_1;                  // we want 1 stop bit (standard)
        USART_InitStruct.USART_Parity = USART_Parity_No;                  // we don't want a parity bit (standard)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
        USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
        USART_Init(USART1, &USART_InitStruct);                                          // again all the properties are passed to the USART_Init function which takes care of all the bit setting
        
        USART_Cmd(USART1, ENABLE);        //Enables USART1
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt
}

//Initiate Interrupts
void init_IRQ(void)
{
	
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//Initiate Interrupt Request on USART 1
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;  //sets the handler for USART1
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //sets the priority, or which interrupt will get called first if multiple interrupts go off at once. The lower the number, the higher the priority.
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;  //sub priority assignment
	NVIC_Init(&NVIC_InitStruct);
	
	
}

void USART1_IRQHandler(void) {
    //Check if interrupt was because data is received
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) 
	{
		uint8_t received = USART_ReceiveData(USART1);
		
		twoPreviousValue = previousValue;
		previousValue = currentValue;
		currentValue = received;
		
		//See if the previous value was a space or a number and if the current value is a number
		if((previousValue == ' ' || (previousValue >= '0' && previousValue <= '9')) && (currentValue >= '0' && previousValue <= '9'))
		{
			tempLaserData[laserSerialCounter] = currentValue;
			laserSerialCounter++;
		}
		else if (previousValue == ',' && (twoPreviousValue >= '0' && twoPreviousValue <= '9'))
		{
			if(currentValue == 'c')
			{
				for(int i = 0; i < (laserSerialCountera); i++)
				{
					if(i == 0)
					{
						laserDataBuff[dataMeasurementCounter] = (tempLaserData[i] - '0');
					}
					else
					{
						laserDataBuff[dataMeasurementCounter] = (laserDataBuff[dataMeasurementCounter] * 10) + (tempLaserData[i] - '0');
					}
				}
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				
				if(laserDataBuff[dataMeasurementCounter] < 200)
				{
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
				}
				
				uint8_t sendData = (laserDataBuff[dataMeasurementCounter] >> 8);
				USART_puts(USART1, sendData);
				
				sendData = (laserDataBuff[dataMeasurementCounter]);
				USART_puts(USART1, sendData);
				
				
				dataMeasurementCounter++;
				
				//Clear tempLaserData
				for(int i = 0; i < laserSerialCounter; i++)
				{
					tempLaserData[i] = 0;
				}
				laserSerialCounter = 0;
			}
			else
			{
				//Clear tempLaserData
				for(int i = 0; i < laserSerialCounter; i++)
				{
					tempLaserData[i] = 0;
				}
				laserSerialCounter = 0;
			}
		
		}
		
	
        //Clear interrupt flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}


int main(void)
{
	init_IRQ();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* Configures the leds and the read/write pin on D1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	init_USART1(115200); 	// initialize USART1 baud rate
	
	while(1)
	{
	
	}
}