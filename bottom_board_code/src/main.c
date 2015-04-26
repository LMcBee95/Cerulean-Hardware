

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

GPIO_InitTypeDef  GPIO_InitStructure;  //Why is this at the top of the code?

#define PACKET_SIZE 16
#define TOP_BOTTOM_BAUD 115200
#define BOTTOM_MOTOR_BAUD 57600

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct c
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

#define SERVO_PERIOD				26250 //* 2  //this users clock 2 for timer 9; clock 2 is 2 times faster than clock 1
#define MAXSERVO 					2.1
#define MINSERVO 					0.8
#define MAXSERVOANGLE 				135.0

#define MAXSERVOCONTROLLED				0
#define MINSERVOCONTROLLED				41

/*** Servo 1 Init ***/

#define SERVO_1_TIMER_CLOCK			RCC_APB1Periph_TIM4
#define SERVO_TIMER_PIN_AF			GPIO_AF_TIM4
#define SERVO_TIMER					TIM4
#define SERVO_1_CLOCK_BANK			RCC_AHB1Periph_GPIOB
#define SERVO_BANK   				GPIOB
#define SERVO_1_PIN					GPIO_Pin_7
#define SERVO_1_PIN_SOURCE			GPIO_PinSource7
#define SERVO_1_CCR					TIM4->CCR2

/*** Servo 2 Init ***/

#define	SERVO_2_PIN 				GPIO_Pin_8
#define SERVO_2_PIN_SOURCE			GPIO_PinSource8
#define SERVO_2_CCR					TIM4->CCR3


uint8_t storage[PACKET_SIZE];	 //Array used to store the data received from the top board
uint8_t motor[8][7];	 //A multidimensional array to store all of the motor commands
uint8_t poll[7]; 		 //An array to store the packet that will poll the motors
volatile uint8_t pollReceived[7]; //An array used to store the packet received from the motors after they are polled
volatile uint8_t reset[7];		 //An array to send a reset command if one of the motors has a fault

volatile uint8_t counter = 0;
volatile uint8_t pollCounter = 0; //Keeps track of how many packets have been sent since we last polled a motor
volatile uint8_t pollAddress = 1; //Stores the address of the motor that is going to be pulled next

void initialize_servo_timer(void)
{
	uint16_t frequency = 50;  //period of 20 ms
	uint16_t preScaler = 64;
	
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(SERVO_1_TIMER_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(SERVO_1_CLOCK_BANK, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = SERVO_1_PIN | SERVO_2_PIN;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(SERVO_BANK, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(SERVO_BANK, SERVO_1_PIN_SOURCE, SERVO_TIMER_PIN_AF);
	GPIO_PinAFConfig(SERVO_BANK, SERVO_2_PIN_SOURCE, SERVO_TIMER_PIN_AF);
	 
	// Compute prescaler value for timebase
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do

	uint16_t PreCalPeriod = ((84000000 / preScaler) / frequency) - 1; 
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM9
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the pre scaler which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(SERVO_TIMER, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(SERVO_TIMER, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	TIM_OC2Init(SERVO_TIMER, &TIM_OCInitStructure);
	 
	// Enable TIM9 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
	 
	// Enable TIM9 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(SERVO_TIMER, ENABLE);
	 
	// Enable TIM9 counter
	TIM_Cmd(SERVO_TIMER, ENABLE); 
	
}

void setServo1Angle(uint8_t angle)
{ 	
	SERVO_1_CCR = (((SERVO_PERIOD + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO ));
}

void setServo2Angle(uint8_t angle)
{ 	
	SERVO_2_CCR = (((SERVO_PERIOD + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO ));
}


void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

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


/* This function initializes USART6 peripheral
 * 
 * Arguments: baud rate --> the baud rate at which the USART is 
 * 						   supposed to operate
 */


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

}

void init_USART6(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	 /*Enable the read write pins*/

	//PinOutput(USART6_ENABLE_PIN, USART6_ENABLE_PORT, USART6_ENABLE_CLK);
    //PinOutput(USART6_DISABLE_PIN, USART6_DISABLE_PORT, USART6_DISABLE_CLK);
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	 /* Configures the leds and the read/write pin on C8 and C9 */
	 GPIO_InitStructure.GPIO_Pin = USART6_ENABLE_PIN | USART6_DISABLE_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOD, &GPIO_InitStructure); 
	 //PinOutput(USART6_ENABLE_PIN, USART6_ENABLE_PORT, USART6_ENABLE_CLK);
     //PinOutput(USART6_DISABLE_PIN, USART6_DISABLE_PORT, USART6_DISABLE_CLK);
	 
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART6 initialization
	
	/* enable APB2 peripheral clock for USART6 
	 * note that only USART6 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART6, PC6 for Rx and PC7 for TX
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
	
	USART_Cmd(USART6, ENABLE);	//Enables USART6
}

//Creates a check sum 
//You pass in the name of the array, and how many bytes you want the checksum to read
uint8_t checksum(volatile uint8_t* packet, uint8_t size) {
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

//Reads through the motor commands and sends them to the motors
void sendPackets(void){
	for(uint8_t i = 0; i < 8; i++)  //Cycles through all of the packets
	{
		for(uint8_t j = 0; j < 7; j++) //Cycles through all of the information in the packets
		{
			USART_puts(USART6, motor[i][j]);
		}
	}
	
}

/*	
 *	Converts the motor commands sent from the top board to packets that the motor controllers can read
 *	You pass the name of the array where you stored the info from the top packet into the function
 */

void convertTBtoBB(uint8_t* top)
{
	//Reads through the motor values from the received top packet
	for (int i = 0; i < 8; i++) 
	{
		
		// 0 for reverse, 1 for forward
		uint8_t direction = (top[i+ 1] < 128);
		
		// Removes the first byte that gave the direction, and 
		//Bit shifts he rest of the number to multiply the value by two to get the values between 0 and 254
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
		motor[i][0] = 0x12;  //This is out of plasce because it gives me errors if I set the start byte value first
	}
	uint16_t servo_angle_temp = (top[9] * (MAXSERVOCONTROLLED - MINSERVOCONTROLLED) / 255) + MINSERVOCONTROLLED;
	uint8_t servo_angle = servo_angle_temp;
	//setServo1Angle(servo_angle); 
	//setServo2Angle(180 - servo_angle);
	if(servo_angle > 100)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
	}
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
		USART_puts(USART6, reset[i]);
}

/*
 *	This function handles receiving the top packet and then sending out the commands to the motor  
 *	controllers. It takes no arguments. The function will keep cycling until it receives a 0x12. Then 
 *	function stores the top board information into an array, produces motor control commands from the 
 *	top packet,and then sends the commands to all of the motors. The function checks before it sends 
 * 	the command to make sure the check sum is correct and that the last byte is a 0x13. If this is 
 * 	true then the function returns a one and if it isn't, then the function returns a 0.
 */

uint8_t handleTopPacket(void) {

    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){
        uint8_t received = USART_ReceiveData(USART2);

        if (received == 0x12) {
            GPIO_SetBits(GPIOD, GPIO_Pin_12);	//Turns on the green led 

            uint8_t timer = 0;	//Timer used to stop the function from waiting for data if there is an error
            uint8_t counter = 1;	//Counter used to count how many bytes we have read in from the top borad

            while (counter < PACKET_SIZE && timer < 0xFFFF) {
            //Waits until all 16 bytes are read in. If no data comes then the function will break 
            //out after a short period of time

                if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) { 
                //if data is received store it in the array storage

                    //Puts data into storage, unless 12, then assume new packet and return
                    received = USART_ReceiveData(USART2);
                    if (received != 0x12) {
                        storage[counter] = received;
                        counter++;
                    } else {
                        GPIO_ResetBits(GPIOD, GPIO_Pin_12); 
                        return(0);
                    }
                }
                timer++;
            }	

            //Checks if checksum is correct and the last byte is 0x13
            if ( (checksum(storage, PACKET_SIZE - 3) == storage[PACKET_SIZE - 2]) && (storage[PACKET_SIZE - 1] == 0x13) ){
              
                //PACKET IS VALID
                
                //Turn on the orange led because packet is valid 
                GPIO_SetBits(GPIOD, GPIO_Pin_13);

                //Take topboard packet and convert to motor controller commmands and send
                convertTBtoBB(storage);
                sendPackets();
                return(1);

            } else {
                //NOT valid packet
                //GPIO_ResetBits(GPIOD, GPIO_Pin_12);
                GPIO_ResetBits(GPIOD, GPIO_Pin_13);
                return(0);  //Returns 0 if the check sum or end byte were incorrect
            }

        } else {
            //Did not receive a 12, so not valid
            //GPIO_ResetBits(GPIOD, GPIO_Pin_12);
            return(0); 
        }

    } else {
        //Stuff isn't set correctly
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        return(0);
    }
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

void init_LEDS(void)
{
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configures the leds and the read/write pin on D1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 | USART6_ENABLE_PIN | USART6_DISABLE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int main(void) {

  init_LEDS(); //initialize the LEDS on the STM32F4 Discovery Board
  init_USART6(BOTTOM_MOTOR_BAUD); 	// initialize USART6 baud rate
  init_USART2(TOP_BOTTOM_BAUD);	// initialize USART2 baud rate
  initialize_servo_timer();

  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);	//Turns the read/write pin for rs485 to write mode
  Delay(0xFFF); //Delays to give the read/write pin time to initialize
  
  uint8_t i = 0;
  while (1)
  {     
		setServo1Angle(10);
		Delay(0xffffff);
		
		
		setServo1Angle(80);
		Delay(0xffffff);

		//Reads the top packet, converts the top packet to motor packets, and then sends the motor packets to the motor controllers
		if(handleTopPacket())  //If the read top packet function was successful, go within the if statement
		{			
			
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			
			//Increments how many times the motor packets have been sent
			pollCounter++;
				
			//Waits for twenty packets to be sent to the motors before polling a motor.
			if(pollCounter > 20)
			{
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
				
				//Sends a packet to poll the motor at pollAddress
				//pollMotor(pollAddress);	
				
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
	  
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);	//Turns off the led 
        GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);		
    }
	
}
