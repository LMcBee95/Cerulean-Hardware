
#include "Bottom_Board_Functions.h"

/********** Global Variables **********/

uint8_t pollingMotors = 0;  //stores a 0 if we are not polling the motors and a 1 if the motors are being polled
uint8_t notPolledCounter = 0;  //stores how many times the bottom board received a top board packet before it received the poll response from the motor controllers

uint8_t motor[8][7];	 //A multidimensional array to store all of the motor commands
uint8_t poll[7]; 		 //An array to store the packet that will poll the motors
uint8_t storage[PACKET_SIZE];  //stores the message the packet that is sent from the top board
uint8_t pollStorage[MOTOR_PACKET_SIZE];

uint8_t pollReceived[7]; //An array used to store the packet received from the motors after they are polled
uint8_t reset[7];		 //An array to send a reset command if one of the motors has a fault
uint8_t counter = 0;
uint8_t pollCounter = 0; //Keeps track of how many packets have been sent since we last polled a motor
uint8_t pollAddress = 1; //Stores the address of the motor that is going to be pulled next
uint8_t received;

uint8_t tempLaserData[10];  //Stores the decimal values of the length of the measurement.
uint16_t laserDataBuff[100];  //Stores the actual value of the length in mm from the target

uint8_t laserSerialCounter = 0;  //Counter that determines where in the tempLaserData to store the next value
uint8_t dataMeasurementCounter = 0;  //Counter that determines which element of laserDataBuff to store the next value 
uint8_t twoPreviousValue = 0;  //The value from two serial readings ago
uint8_t previousValue = 0;    //The value from the last serial reading
uint8_t currentValue = 0;     //The current value from the serial

uint16_t ADC1ConvertedValue[NUM_DMA_2_CONVERSIONS];

GPIO_InitTypeDef  GPIO_InitStructure;  //this is used by all of the pin initiations, must be included

/********** Function Definitions **********/

void anologWrite(uint32_t channel, uint8_t dutyCycle, uint32_t period)
{
	TIM3->CCR4 = (period + 1) * dutyCycle / 255.0;
}

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
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) 
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
	for(uint8_t i = 0; i < MOTOR_PACKET_SIZE; i++)
		USART_puts(USART6, poll[i]);
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
	for(uint8_t i = 0; i < MOTOR_PACKET_SIZE; i++)
		USART_puts(USART6, reset[i]);
}

void sendPackets(void){
	for(uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)  //Cycles through all of the packets
	{
		for(uint8_t j = 0; j < MOTOR_PACKET_SIZE; j++) //Cycles through all of the information in the packets
		{
			USART_puts(USART6, motor[i][j]);
		}
	}
}


void setServo1Angle(uint8_t angle)
{ 	
	SERVO_1_CCR = (((SERVO_PERIOD + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO ));
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
				for(int i = 0; i < (laserSerialCounter); i++)
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
				
				//Used for testing purposes
				if(laserDataBuff[dataMeasurementCounter] < 200)
				{
					GREEN_LED_ON
				}
				
				uint8_t sendData = (laserDataBuff[dataMeasurementCounter] >> 8);
				USART_puts(LASER_USART	, sendData);
				
				sendData = (laserDataBuff[dataMeasurementCounter]);
				USART_puts(LASER_USART	, sendData);
				
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
        USART_ClearITPendingBit(LASER_USART	, USART_IT_RXNE);
    }
}

void USART2_IRQHandler(void) {
    //Check if interrupt was because data is received
    if (USART_GetITStatus(USART2, USART_IT_RXNE)) 
	{	
		received = USART_ReceiveData(USART2);
	
		if(received == START_BYTE)
		{
			
			storage[counter] = received;
			counter = 1;
		}
		else if(counter > 0 && received != START_BYTE)
		{
			storage[counter] = received;
			counter++;
			
			if(counter == PACKET_SIZE  && (checksum(storage, PACKET_SIZE - 3) == storage[PACKET_SIZE - 2]) && (storage[PACKET_SIZE - 1] == END_BYTE))
			{
				RED_LED_ON
				convertTBtoBB(storage);  //Converts the data from the top board into motor controller commands that we can use
				
				if(!pollingMotors)  //if we are not polling the motors for fault data, pollingMotors will be 0 and the the code will send motor commands to the motor controllers
				{
					
					
					sendPackets();	//Sends the motor controller commands produced by the convert function
					pollCounter++;
				
					if(pollCounter > 20)
					{
						
						pollMotor(pollAddress);
						
						pollingMotors = 1;
						
						Delay(0x3FFF);		//Wait for the read write pin to turn low
						//GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
						//GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
						
						pollAddress++;
						if(pollAddress == 9)
						{
							pollAddress = 1;
						}
						pollCounter = 0;  //Resets the poll counter

					}
				}
				else
				{
					notPolledCounter++;
					
					if(notPolledCounter > POLL_MOTOR_TIME_OUT)
					{
						GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
						GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
						pollingMotors = 0;
						notPolledCounter = 0;
						
						//print error message or send some message to the top board.
					}
				}
				counter = 0; //Reset the counter
				
			}
			else if(counter == PACKET_SIZE)
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			}
		}
		else
		{
			counter = 0;
		}
	}
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}

void USART6_IRQHandler(void) {
    
	
	RED_LED_OFF
	//Check if interrupt was because data is received
	if (USART_GetITStatus(USART6, USART_IT_RXNE)  && 0) {
		received = USART_ReceiveData(USART6);
		
		if(received == START_BYTE)
		{
			pollStorage[pollCounter] = received;
			pollCounter = 1;
		}
		else if(pollCounter > 0 && received != START_BYTE)
		{
			pollStorage[pollCounter] = received;
			pollCounter++;
			
			if(pollCounter == MOTOR_PACKET_SIZE  && (checksum(pollStorage, MOTOR_PACKET_SIZE - 3) == pollStorage[MOTOR_PACKET_SIZE - 2]) && (pollStorage[MOTOR_PACKET_SIZE - 1] == END_BYTE))
			{
				//do stuff with the received data
				
				ORANGE_LED_ON
				
				pollCounter = 0; //Reset the counter
				
				
				pollingMotors = 0;  //resets the variable that will allow the board to start sending out motor commands again
				
				GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
				GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
				
			}
			else if(pollCounter == MOTOR_PACKET_SIZE)
			{
				//GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			}
		}
		else
		{
			pollCounter = 0;
		}
	}
        
        //Clear interrupt flag
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}


void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

/********** Initializations *********/

void init_DMA(uint16_t *array, uint16_t size)
{
	ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;
    GPIO_InitTypeDef      GPIO_InitStruct;
    /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ADC1 is connected to the APB2 peripheral bus
    /* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStruct.DMA_Channel = DMA_Channel_2;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//ADC3's data register
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)array; //specifies where the memory goes when it is received
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = size;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream0, ENABLE);
    
	/* Configure GPIO pins ******************************************************/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4, GPIO_Pin_5;// PC0, PC1, PC2, PC3, PC4, PC5
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down
    GPIO_Init(GPIOC, &GPIO_InitStruct);//Initialize GPIOC pins with the configuration
	
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;//PA4, PA5
    GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration
	
    
	/* ADC Common Init **********************************************************/
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);
    
	/* ADC1 Init ****************************************************************/
    ADC_DeInit();
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = size;
    ADC_Init(ADC1, &ADC_InitStruct);//Initialize ADC with the configuration
    
	/* Select the channels to be read from **************************************/
    
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_144Cycles);//PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  2, ADC_SampleTime_144Cycles);//PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_144Cycles);//PC0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_144Cycles);//PC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_144Cycles);//PC4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 8, ADC_SampleTime_144Cycles);//PC5
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);
}

void init_IRQ(void)
{
	/*
		Interrupt Priorities
		0 : USART 2
		1 : USART 6	
		2 : USART 1
	*/
	
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//Initiate Interrupt Request on USART channel 2
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	//Initiate Interrupt Request for USART  channel 6
	NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	//Initiate Interrupt Request for USART  channel 1
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;  //sets the handler for USART1
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //sets the priority, or which interrupt will get called first if multiple interrupts go off at once. The lower the number, the higher the priority.
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;  //sub priority assignment
	NVIC_Init(&NVIC_InitStruct);
}

void init_LEDS(void)
{
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configures the LEDS*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

uint16_t initialize_servo_timer(void)
{
	uint16_t frequency = 50;  //period of 20 ms
	uint16_t preScaler = 64;
	
	// Enable TIM3 and GPIOC clocks
	RCC_APB2PeriphClockCmd(SERVO_1_TIMER_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(SERVO_1_CLOCK_BANK, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = SERVO_1_PIN;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(SERVO_1_BANK, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(SERVO_1_BANK, SERVO_1_PIN_SOURCE, SERVO_1_TIMER_PIN_AF);

	 
	// Compute prescaler value for timebase
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do

	uint16_t PreCalPeriod = ((84000000 / preScaler) / frequency) - 1; 

	if(PreCalPeriod == SERVO_PERIOD)
	{
		GREEN_LED_ON
	}
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM9
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the pre scaler which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(SERVO_1_TIMER, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(SERVO_1_TIMER, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(SERVO_1_TIMER, TIM_OCPreload_Enable);
	 
	// Enable TIM3 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(SERVO_1_TIMER, ENABLE);
	 
	// Enable TIM3 counter
	TIM_Cmd(SERVO_1_TIMER, ENABLE); 
	
	return(PreCalPeriod);
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
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt
}

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

void init_USART6(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	 /*Enable the read write pins*/
	
	
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 GPIO_InitTypeDef GPIO_InitStructure;

	 /* Configures the leds and the read/write pin on C8 and C9 */
	 GPIO_InitStructure.GPIO_Pin = USART6_ENABLE_PIN | USART6_DISABLE_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(USART6_DISABLE_PORT, &GPIO_InitStructure); 
	 
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
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt
}

int32_t initialize_pwm_timers(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM3
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	//TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	//TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	//TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	 
	// Enable TIM3 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	 
	// Enable TIM3 counter
	TIM_Cmd(TIM3, ENABLE); 
	
	return(PreCalPeriod);
}
