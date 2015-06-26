
#include "Bottom_Board_Functions.h"
#include "stepper.h"

/******************** Global Variables ********************/

/*** Serial Communication ***/

uint8_t pollingMotors = 0;  //stores a 0 if we are not polling the motors and a 1 if the motors are being polled
uint8_t notPolledCounter = 0;  //stores how many times the bottom board received a top board packet before it received the poll response from the motor controllers

uint8_t motor[8][7];	 //A multidimensional array to store all of the motor commands
uint8_t poll[7]; 		 //An array to store the packet that will poll the motors
uint8_t storage[PACKET_SIZE];  //stores the message that is sent from the top board
uint8_t pollStorage[MOTOR_PACKET_SIZE];
uint8_t dataGoingUp[SENT_PACKET_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //Storage for the data that is going to be sent up to the base-station

uint8_t pollReceived[7]; //An array used to store the packet received from the motors after they are polled
uint8_t reset[7];		 //An array to send a reset command if one of the motors has a fault
uint8_t counter = 0;
uint8_t pollCounter = 0; //Keeps track of how many packets have been sent since we last polled a motor
uint8_t pollAddress = 1; //Stores the address of the motor that is going to be pulled next
uint8_t received;  //Variable to store in incoming serial data

/*** Variables for Stepper Motors ***/
Stepper* horizontalStepper;  //Structure to store horizontal stepper data
Stepper* verticalStepper;    //Structure to store vertical stepper data

/*** Variables for Laser Tool ***/

uint8_t tempLaserData[10];  //Stores the decimal values of the length of the measurement.
uint16_t laserDataBuff[1000];  //Stores the actual value of the length in mm from the target

uint8_t laserSerialCounter = 0;  //Counter that determines where in the tempLaserData to store the next value
uint8_t dataMeasurementCounter = 0;  //Counter that determines which element of laserDataBuff to store the next value 
uint8_t twoPreviousValue = 0;  //The value from two serial readings ago
uint8_t previousValue = 0;    //The value from the last serial reading
uint8_t currentValue = 0;     //The current value from the serial

int laserIsOn = 0;

/*** Time Function ***/
uint32_t time = 0;  //Keeps track of the number of ms that the program has been running for (for time update look at TIM5_IRQHandler)



/** stepper step trackers **/
uint8_t vertSteps = 0;
uint8_t horrSteps = 0;

GPIO_InitTypeDef  GPIO_InitStructure;  //this is used by all of the pin initiations, must be included

/******************** Function Definitions ********************/

/******************************************************************************
* Description: <Sets the speed of the bilge pump in one direction.>
* Parameters:  <bilgePumpSpeeed; uint8_t;  whether the bilge pump should be on or not>             
******************************************************************************/
void bilgePumpPwm(uint8_t bilgePumpOn)
{
	if(bilgePumpOn > 0)
	{
		TIM10->CCR1 = (GENERAL_PWM_PERIOD) * MAX_BILGE_PUMP_VALUE / 255;
		TIM11->CCR1 = 0;
	}
	else
	{
		TIM10->CCR1 = 0;
		TIM11->CCR1 = 0;
	}
	
}

/******************************************************************************
* Description: <Sets the brightness of the five head light leds on the rov.>
* Parameters:  <led1DutyCycle; uint8_t; led1 brightness>      
*			   <led2DutyCycle; uint8_t; led2 brightness>
*			   <led3DutyCycle; uint8_t; led3 brightness>
*			   <led4DutyCycle; uint8_t; led4 brightness>
*			   <led5DutyCycle; uint8_t; led5 brightness>
******************************************************************************/
void cameraLedPwm(uint8_t led1DutyCycle, uint8_t led2DutyCycle, uint8_t led3DutyCycle, uint8_t led4DutyCycle, uint8_t led5DutyCycle)
{
	TIM2->CCR4 = (LED_PWM_PERIOD) * led1DutyCycle / 255.0;
	TIM2->CCR3 = (LED_PWM_PERIOD) * led2DutyCycle / 255.0;
	TIM2->CCR2 = (LED_PWM_PERIOD) * led3DutyCycle / 255.0;
	TIM14->CCR1 = (LED_PWM_PERIOD) * led4DutyCycle / 255.0;
	TIM13->CCR1 = (LED_PWM_PERIOD) * led5DutyCycle / 255.0;
}

/******************************************************************************
* Description: <Calculates a checksum from a packet of data to ensure no data has become corrupted.>
* Parameters:  <packet; uint8_t*;  The starting adress of the array containing the packet>    
*			   <size; uint8_t;  The size of the array with the packet>   
******************************************************************************/
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

/******************************************************************************
* Description: <Sets the two pwm signals needed to control the claw h-bridge. 
*				One of the pwm will be zero while the other is between zero 
*				and 255.>
* Parameters:  <PWM_IN1; uint8_t;  pwm for the first pin>    
*			   <PWM_IN2; uint8_t;  pwm for the second pin>  
******************************************************************************/ 
void clawPwm(uint8_t PWM_IN1, uint8_t PWM_IN2)
{
	TIM10->CCR1 = (GENERAL_PWM_PERIOD) * PWM_IN2 / 255.0;	
	TIM11->CCR1 = (GENERAL_PWM_PERIOD) * PWM_IN1 / 255.0;	
}

/******************************************************************************
* Description: <Converts motor commands from the tether into commands that
*				can be sent to the motors.>
* Parameters:  <top; uint8_t*;  address of the array that contains the data 
*				from the tether>    
******************************************************************************/ 
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

/******************************************************************************
* Description: <Loops through a while loop to create a delay.>
* Parameters:  <nCount; uint32_t;  The number of clock cycles the user wants
*   			to delay the cpu for; the larger the value the longer the 
*				delay is>    
******************************************************************************/ 
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/******************************************************************************
* Description: <Sends out a command to poll the motors for faults.>
* Parameters:  <address; uint8_t;  address of the motor that needs to be 
*				pulled>    
******************************************************************************/ 
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
		USART_puts(UART5, poll[i]);
}

/******************************************************************************
* Description: <Sends out a command to reset a motor.>
* Parameters:  <address; uint8_t;  address of the motor that needs to be reset>    
******************************************************************************/ 
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
		USART_puts(UART5, reset[i]);
}

/******************************************************************************
* Description: <Sets the brightness of each of the party leds.>
* Parameters:  <address; uint8_t;  address of the motor that needs to be pulled>    
******************************************************************************/ 
void RGBLedPwm(uint8_t dutyCycleRed, uint8_t dutyCycleGreen, uint8_t dutyCycleBlue)
{
	TIM4->CCR1 = (GENERAL_PWM_PERIOD) * dutyCycleBlue / 255.0;
	TIM4->CCR2 = (GENERAL_PWM_PERIOD) * dutyCycleGreen / 255.0;
	TIM4->CCR3 = (GENERAL_PWM_PERIOD) * dutyCycleRed / 255.0;
	
}

/******************************************************************************
* Description: <Sends data from the micro board to the base station>
* Parameters:  <none>    
******************************************************************************/ 
void sendDataUp(void)
{

	//RGBLedPwm(255, 255, 255);
	dataGoingUp[0] = START_BYTE;
	dataGoingUp[SENT_PACKET_SIZE - 1] = END_BYTE;
	dataGoingUp[SENT_PACKET_SIZE - 2] = checksum(dataGoingUp, SENT_PACKET_SIZE - 3);
	
	for(uint8_t i = 0; i < SENT_PACKET_SIZE; i++)
	{
		USART_puts(USART6, dataGoingUp[i]);
	}
}

/******************************************************************************
* Description: <Sends data from the micro board to all eight motors>
* Parameters:  <none>    
******************************************************************************/ 
void sendPackets(void){
	for(uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)  //Cycles through all of the packets
	{
		for(uint8_t j = 0; j < MOTOR_PACKET_SIZE; j++) //Cycles through all of the information in the packets
		{
			USART_puts(UART5, motor[i][j]);
		}
	}
}

/******************************************************************************
* Description: <Takes a command from the user and transfers that command to the
*				laser distance measurement tool>
* Parameters:  <command; char*; command that the user wants to send to the 
*				laser measurement tool>    
******************************************************************************/ 
void sendLaserCommand(char* command)
{
	
	uint8_t index = 0;
	char letter = command[index];

	while(letter != '#')
	{
		USART_puts(LASER_USART, letter);
		index++;
		letter = command[index];
	}
	
	USART_puts(LASER_USART, '#');
	
}

/******************************************************************************
* Description: <Sets the angle of stepper motor one to the angle given by the 
*				user>
* Parameters:  <angle; uint8_t; angle given by the user>  
******************************************************************************/ 
void setServo1Angle(uint8_t angle)
{ 	
	SERVO_1_CCR = (((SERVO_PERIOD + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO ));
}

/******************************************************************************
* Description: <Sets the angle of stepper motor two to the angle given by the 
*				user>
* Parameters:  <angle; uint8_t; angle given by the user>  
******************************************************************************/
void setServo2Angle(uint8_t angle)
{ 	
	SERVO_2_CCR = (((SERVO_PERIOD + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO ));
}

/******************************************************************************
* Description: <Takes the data from the packet sent from the base station and
*				and changes the angle of the stepper motor accordingly>
* Parameters:  <none>  
******************************************************************************/
void setSteppers(void)
{
	uint8_t  byte = storage[STEPPER_DOWN_BYTE]; //Get surface packet
	uint8_t angles = Stepper_UseByte(byte, horizontalStepper, verticalStepper); //Move steppers and get angles
  
	//Writes angle data to the up packet
	//dataGoingUp[STEPPER_UP_VERT] = Stepper_GetStep(verticalStepper) && 0xFF;      
	//dataGoingUp[STEPPER_UP_HOR] = Stepper_GetStep(horizontalStepper) && 0xFF;
}

/******************************************************************************
* Description: <Function used for testing purposes for the stepper motor>
* Parameters:  <byte; uint8_t; the number of steps to update the stepper motor>  
******************************************************************************/
void setSteppersDebugByte(uint8_t byte)
{
	storage[STEPPER_DOWN_BYTE] = byte;
}

void stepperPwm(uint8_t dutyCycle1, uint8_t dutyCycle2)
{
	TIM12->CCR1 = (GENERAL_PWM_PERIOD) * dutyCycle1 / 255.0;	
	TIM12->CCR2 = (GENERAL_PWM_PERIOD) * dutyCycle2 / 255.0;	
}

/******************************************************************************
* Description: <Interrupt handler for timer 5; This timer is called 
*				approximately every 1 ms; handles updating the stepper motors
*				and triggering the sending of data back up to the base 
*				station>
* Parameters:  <none>  
******************************************************************************/
void TIM5_IRQHandler(void)
{

 if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
 {

	 
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	time++; //Updates the current time that the program has been running
	
	if(time % 500 == 0)
	{
		//dataGoingUp[2] = ADC1ConvertedValue[VOLT_48_CURRENT] & 0xff;
		turnFootPwm(0, 0);
		
		//sendDataUp();
		
		//sendUpTrigger = 1;
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	}
	if(verticalStepper->stepBuffer > 0) Stepper_Enable(verticalStepper);
	if(horizontalStepper->stepBuffer > 0) Stepper_Enable(horizontalStepper);
	
	
	Stepper_Update(verticalStepper);
	Stepper_Update(horizontalStepper);
	
	Stepper_Disable(verticalStepper);
	Stepper_Disable(horizontalStepper);
 }

}

/******************************************************************************
* Description: <Sets the speed of the turning foot DC  motor>
* Parameters:  <PWM_IN1; uint8_t; When PWM_IN2 is held at 0, this parameters
*				acts as the speed of the turning foot motor in the clockwise
*				direction>  
*				<PWM_IN2; uint8_t; When PWM_IN1 is held at 0, this parameters
*				acts as the speed of the turning foot motor in the counter-
*				clockwise direction>
******************************************************************************/
void turnFootPwm(uint8_t PWM_IN1, uint8_t PWM_IN2)
{
	TIM3->CCR1 = (GENERAL_PWM_PERIOD) * PWM_IN1 / 255.0;	
	TIM3->CCR2 = (GENERAL_PWM_PERIOD) * PWM_IN2 / 255.0;	
}

/******************************************************************************
* Description: <Interrupt handler for USART2; deals with parsing the distance	
*				data sent back from the laser measurement tool>
* Parameters:  <none>
******************************************************************************/
void USART2_IRQHandler(void) {
    //Check if interrupt was because data is received
    if (USART_GetITStatus(USART2, USART_IT_RXNE)) 
	{
		uint8_t received = USART_ReceiveData(USART2);
		
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
				for(uint8_t i = 0; i < (laserSerialCounter); i++)
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

				/*******************
				
				The data for the laser measurement tool is stored within an array called
				laserDataBuff where dataMeasurement counter corresponds to the last 
				measurement read in.  The data for the measurement is stored as an
				unsigned 16 bit integer.  Below This I have included some code where I
				have successfully sent the 16 bit packet using USART.  
				
				
				uint8_t sendData = (laserDataBuff[dataMeasurementCounter] >> 8);
				USART_puts(LASER_USART	, sendData);
				
				sendData = (laserDataBuff[dataMeasurementCounter]);
				USART_puts(LASER_USART	, sendData);
				
				*******************/
				
				dataGoingUp[6] = laserDataBuff[dataMeasurementCounter] >> 8;  //The 8 most significant bits of the distance meansurement
				dataGoingUp[7] = laserDataBuff[dataMeasurementCounter] && 0xFF; //The 8 least significant bits fo the distance measurement
				
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

/******************************************************************************
* Description: <Interrupt handler for UART5; deals with the communication of 
*				data between the micro board and the eight thruster motors>
* Parameters:  <none>
******************************************************************************/
void UART5_IRQHandler(void) {
    
	//Check if interrupt was because data is received
	if (USART_GetITStatus(UART5, USART_IT_RXNE)) {
		received = USART_ReceiveData(UART5);
		
		if(received == START_BYTE)
		{
			//RGBLedPwm(255, 255, 255);
			pollStorage[pollCounter] = received;
			pollCounter = 1;
			GPIO_SetBits(GPIOD, GPIO_Pin_13); 
		}
		else if(pollCounter > 0 && received != START_BYTE)
		{
			pollStorage[pollCounter] = received;
			pollCounter++;
			
			if(pollCounter == MOTOR_PACKET_SIZE  && (checksum(pollStorage, MOTOR_PACKET_SIZE - 3) == pollStorage[MOTOR_PACKET_SIZE - 2]) && (pollStorage[MOTOR_PACKET_SIZE - 1] == END_BYTE))
			{
				//do stuff with the received data
				
				
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
			pollCounter = 0;//Clear interrupt flag
			
		}
		USART_ClearITPendingBit(UART5, USART_IT_RXNE); 
	}
}

/******************************************************************************
* Description: <Interrupt handler for USART2; deals with handling the data 
*				received from the base station and parsing that data>
* Parameters:  <none>
******************************************************************************/
void USART6_IRQHandler(void) {
    
	 //Check if interrupt was because data is received
    if (USART_GetITStatus(USART6, USART_IT_RXNE)) 
	{	
		received = USART_ReceiveData(USART6);
		
		//if the byte is the start byte, 0x12, then the code triggers
		if(received == START_BYTE)
		{
			storage[counter] = received;
			counter = 1;
			
			
		}
		else if(counter > 0 && received != START_BYTE)
		{
			storage[counter] = received;
			counter++;
			
			
			//checks if  all the bytes have been recieved, if they are, it then checks if the checksum is correct and if the last byte is 0x13
			if(counter == PACKET_SIZE  && (checksum(storage, PACKET_SIZE - 3) == storage[PACKET_SIZE - 2]) && (storage[PACKET_SIZE - 1] == END_BYTE))
			{
				//command if the base station wants actions to run like they normaly do
				if(COMMAND = NORMAL)
				{
					GPIO_ToggleBits(GPIOD, GPIO_Pin_12); //Indication that the micro board is receiving information
					convertTBtoBB(storage);  //Converts the data from the top board into motor controller commands that we can use
					
					
					/*** Do stuff with the info from the top board ***/
					
					if(ADC3ConvertedValue[VOLT_48_CURRENT] > 3000) // <<figure out what is a good value
					{
						//TODO
					}
					
					
					//gets the angles sent from the top board and converts them into stepper commands
					setSteppers();
					
					/*uint8_t horzDir = byte>>7&0x01;          //Horizontal direction
					uint8_t vertDir = (byte>>3)&0x01;   //Vertical direction
					vertSteps += (byte>>4)&0x07; //Horizontal steps
					horrSteps += byte & 0x07;    //Vertical steps*/
					
					
					//Set camera leds
					cameraLedPwm(LED1_VALUE, LED2_VALUE, LED3_VALUE, LED4_VALUE, LED5_VALUE);
					
					//sets the claw to the correct position
					if(SERVO_CLOSED)
					{
						setServo1Angle(OFF_ANGLE);
						setServo2Angle(180 - OFF_ANGLE);	
					}
					else
					{
						setServo1Angle(ON_ANGLE);
						setServo2Angle(180 - ON_ANGLE);	
					}
					
					if(READ_LASER && !laserIsOn)
					{
						GPIO_SetBits(GPIOD, GPIO_Pin_10);
						laserIsOn = 1;
					}
					
					if(!READ_LASER && laserIsOn)
					{
						GPIO_ResetBits(GPIOD, GPIO_Pin_10);
						laserIsOn = 0;
					}
					
					
					//sets the camera muxes
					if(MUX1)
						GPIO_SetBits(GPIOF, GPIO_Pin_0);
					else
						GPIO_ResetBits(GPIOF, GPIO_Pin_0);
					if(MUX2)
						GPIO_SetBits(GPIOF, GPIO_Pin_1);
					else
						GPIO_ResetBits(GPIOF, GPIO_Pin_1);
						
					
					//Turns the bilge pump either on or off
					bilgePumpPwm(BILGE_PUMP_VALUE);
					
					//sets the speed of the turning foot motor 
					if(FOOT_TURNER_VALUE < 128) //Going Forward
					{
						uint8_t turnFootMag = (FOOT_TURNER_VALUE << 1);
						turnFootPwm(turnFootMag, 0);
					}
					else //Going in Reverse
					{
						uint8_t turnFootMag = (FOOT_TURNER_VALUE  << 1);
						turnFootPwm(0, turnFootMag);
					}

					
					//reads the voltage sensors and outputs the which voltages are high
					if(READ_VOLTAGES || 1)
					{
						
						dataGoingUp[1] = ADC3ConvertedValue[VSEN1] >> 4;
						dataGoingUp[2] = ADC3ConvertedValue[VSEN2] >> 4;
						dataGoingUp[3] = ADC3ConvertedValue[VSEN3] >> 4;
						
						/*
						if(ADC3ConvertedValue[VSEN1] < ADC_TO_VOLTS * ON_VOLTAGE)
							dataGoingUp[1] = 255;
						else
							dataGoingUp[1] = 0;
						
						if(ADC3ConvertedValue[VSEN2] < ADC_TO_VOLTS * ON_VOLTAGE)
							dataGoingUp[2] = 255;
						else
							dataGoingUp[2] = 0;
						
						if(ADC3ConvertedValue[VSEN3] < ADC_TO_VOLTS * ON_VOLTAGE)
							dataGoingUp[3] = 255;
						else
							dataGoingUp[3] = 0;
						*/
					}
					
					/*** End doing stuff with the info from the top board ***/
				}
				
				if(!pollingMotors)  //if we are not polling the motors for fault data, pollingMotors will be 0 and the the code will send motor commands to the motor controllers
				{
					//pollCounter++;
				
					if(pollCounter > 20)
					{
						pollMotor(pollAddress);
						
						pollingMotors = 1;
						
						Delay(0x3FFF);		//Wait for the read write pin to turn low
						GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);  //sets the rs485 on the bottom board to read the response from polling the motors
						GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
						
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
				counter = 0; //Reset the counter
			}
			else if(counter == PACKET_SIZE)
			{
				
			}
		}
		else
		{
			counter = 0;
		}
	}
	USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}

/******************************************************************************
* Description: <Allows the user to send an 8 bit integer over serial>
* Parameters:  <USARTx; USART_TypeDef*; The usart the user of the function
*				wishes to send data across>
*			   <data; uint8_t; the data that the user wants to send by using
*			    serial.
******************************************************************************/
void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

/********** Initializations *********/

/******************************************************************************
* Description: <Initializes timer 11 in order to generate pwm signals for 
*				servo number one>
* Parameters:  <frequency; uint32_t; the number of times the pwm cycle happens
*				per second>
*			   <preScaler; uint16_t; Determines the speed of the clock for 
*				timer 11; the frequency of the timer will be the clock speed
*				of the main processor divided by the pre-scaler>
******************************************************************************/
void initialize_claw1_timer(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM11 and GPIOF clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PF7 pin as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM11 functions.
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_TIM11);

	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM11
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM11
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation

	 
	// Enable TIM11 peripheral Preload register on CCR1 
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);

	 
	// Enable TIM11 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM11, ENABLE);
	 
	// Enable TIM11 counter
	TIM_Cmd(TIM11, ENABLE); 
}

/******************************************************************************
* Description: <Initializes timer 10 in order to generate pwm signals for 
*				servo number one>
* Parameters:  <frequency; uint32_t; the number of times the pwm cycle happens
*				per second>
*			   <preScaler; uint16_t; Determines the speed of the clock for 
*				timer 11; the frequency of the timer will be the clock speed
*				of the main processor divided by the pre-scaler>
******************************************************************************/
void initialize_claw2_timer(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM10 and GPIOF clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PF6 pin as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM10 functions.
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);

	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM10
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM10
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation

	 
	// Enable TIM10 peripheral Preload register on CCR1
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);

	 
	// Enable TIM10 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM10, ENABLE);
	 
	// Enable TIM10 counter
	TIM_Cmd(TIM10, ENABLE); 
}

/******************************************************************************
* Description: <Initializes DMA(direct memory access) for ADC values for ACD
*				channel 1>
* Parameters:  <array; uint16_t *; An array that holds the values of the ADC
*				values to be stored using DMA>
*			   <size; uint16_t; the number of values on this DMA channel
*				that will be read into the buffer>
******************************************************************************/
void init_DMA_ADC1(uint16_t *array, uint16_t size)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
 
  GPIO_StructInit(&GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  DMA_StructInit(&DMA_InitStructure);
 
  /**
    Set up the clocks are needed for the ADC
  */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

 
  /* Analog channel configuration */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /**
    Configure the DMA
  */
  //==Configure DMA2 - Stream 4
  DMA_DeInit(DMA2_Stream4);  //Set DMA registers to default values
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //Source address
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC1ConvertedValue; //Destination address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = size; //Buffer size
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure); //Initialize the DMA
  DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4
 
   /**
    * Config the ADC1 
    **/
   ADC_DeInit();
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_NbrOfConversion = size;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
   ADC_Init(ADC1,&ADC_InitStructure);
 
   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInit(&ADC_CommonInitStructure);
 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_144Cycles);//PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_144Cycles);//PC0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 3, ADC_SampleTime_144Cycles);//PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_144Cycles);//PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 5, ADC_SampleTime_144Cycles);//PC3
   
 
   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
 
   ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA
 
   ADC_Cmd(ADC1, ENABLE);   // Enable ADC1
 
   ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion
}

/******************************************************************************
* Description: <Initializes DMA(direct memory access) for ADC values for ACD
*				channel 3>
* Parameters:  <array; uint16_t *; An array that holds the values of the ADC
*				values to be stored using DMA>
*			   <size; uint16_t; the number of values on this DMA channel
*				that will be read into the buffer>
******************************************************************************/
void init_DMA_ADC3(uint16_t *array, uint16_t size)
{
	ADC_InitTypeDef       ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	DMA_InitTypeDef       DMA_InitStruct;
	GPIO_InitTypeDef      GPIO_InitStruct;
    
	
	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//ADC3 is connected to the APB2 peripheral bus
    
	/* Analog channel configuration : PF3, 4, 5, 10*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;// | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	
	

	
	/* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStruct.DMA_Channel = DMA_Channel_2;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
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
    
    
    /* ADC Common Init **********************************************************/
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);
   


/* ADC3 Init ****************************************************************/
    //ADC_DeInit();
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = size;
    ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration
    /* Select the channels to be read from **************************************/
    

    
	ADC_RegularChannelConfig(ADC3, ADC_Channel_9,  1, ADC_SampleTime_144Cycles);//PF3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_15,  2, ADC_SampleTime_144Cycles);//PF4
	ADC_RegularChannelConfig(ADC3, ADC_Channel_14,  3, ADC_SampleTime_144Cycles);//PF5
	//ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 4, ADC_SampleTime_144Cycles);//PF10
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
   

    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);
    
    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
    
    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);
}

/******************************************************************************
* Description: <Initializes all interrupt handlers used>
* Parameters:  <none>
******************************************************************************/
void init_IRQ(void)
{
	/**
		Interrupt Priorities
		0 : USART 6
		1 : UART  5	
		2 : USART 2
		3 : Timer 5
	*/
	
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//Initiate Interrupt Request on USART channel 2
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStruct);
	
	//Initiate Interrupt Request for USART  channel 6
	NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	//Initiate Interrupt Request for UART  channel 5
	NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;  //sets the handler for USART1
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  //sets the priority, or which interrupt will get called first if multiple interrupts go off at once. The lower the number, the higher the priority.
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;  //sub priority assignment
	NVIC_Init(&NVIC_InitStruct);
	
    /* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void init_LEDS(void)
{
  /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/******************************************************************************
* Description: <Initializes pwm for the five camera leds>
* Parameters:  <frequency; uint32_t; the number of times the pwm happens per 
*				second>
*			   <preScaler; uint16_t; Determines the speed of the clock for 
*				timer 11; the frequency of the timer will be the clock speed
*				of the main processor divided by the pre-scaler>
******************************************************************************/
void initialize_led_timers(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM2 and GPIOA clocks
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//NOT TESTED
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PA1 - PA3 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = LED_PIN1 | LED_PIN2 | LED_PIN3;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(LED_1_2_3_BANK, &GPIO_InitStructure);	//initializes the structure
	
	
	//initializes the led 4 pin
	GPIO_InitStructure.GPIO_Pin = LED_PIN4;
	GPIO_Init(LED_4_BANK, &GPIO_InitStructure);	
	
	//NOT TESTED
	GPIO_InitStructure.GPIO_Pin = LED_PIN5;
	GPIO_Init(LED_5_BANK, &GPIO_InitStructure);	//initializes led4 structure
	
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM2 functions.
	GPIO_PinAFConfig(LED_1_2_3_BANK, LED_SOURCE_PIN1, LED_1_2_3_AF);
	GPIO_PinAFConfig(LED_1_2_3_BANK, LED_SOURCE_PIN2, LED_1_2_3_AF);
	GPIO_PinAFConfig(LED_1_2_3_BANK, LED_SOURCE_PIN3, LED_1_2_3_AF);
	
	//NOT TESTED
	GPIO_PinAFConfig(LED_4_BANK, LED_SOURCE_PIN4, LED_4_AF);
	GPIO_PinAFConfig(LED_5_BANK, LED_SOURCE_PIN5, LED_5_AF);
	
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM2
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_1_2_3_TIMER, &TIM_TimeBaseStructure);  //initializes this part of the code
	
	//NOT TESTED
	TIM_TimeBaseInit(LED_4_TIMER, &TIM_TimeBaseStructure);  //initializes led 4
	TIM_TimeBaseInit(LED_5_TIMER, &TIM_TimeBaseStructure);  //initializes led 5
	
	// Initialize TIM2 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	//initiates this part of the pulse width modulation
	TIM_OC2Init(LED_1_2_3_TIMER, &TIM_OCInitStructure);
	TIM_OC3Init(LED_1_2_3_TIMER, &TIM_OCInitStructure);
	TIM_OC4Init(LED_1_2_3_TIMER, &TIM_OCInitStructure);
	
	
	TIM_OC1Init(LED_4_TIMER, &TIM_OCInitStructure);
	TIM_OC1Init(LED_5_TIMER, &TIM_OCInitStructure);
	
	 
	// Enable TIM2 peripheral Preload register on CCR1 for 4 channels
	
	TIM_OC2PreloadConfig(LED_1_2_3_TIMER, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(LED_1_2_3_TIMER, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(LED_1_2_3_TIMER, TIM_OCPreload_Enable);
	
	
	TIM_OC1PreloadConfig(LED_4_TIMER, TIM_OCPreload_Enable);  //sets up led4 for pwm
	TIM_OC1PreloadConfig(LED_5_TIMER, TIM_OCPreload_Enable);  //sets up led5 for pwm
	 
	// Enable TIM2 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(LED_1_2_3_TIMER, ENABLE);
	
	
	TIM_ARRPreloadConfig(LED_4_TIMER, ENABLE);
	TIM_ARRPreloadConfig(LED_5_TIMER, ENABLE);
	 
	// Enable TIM2 counter
	TIM_Cmd(LED_1_2_3_TIMER, ENABLE); 
	
	
	TIM_Cmd(LED_4_TIMER, ENABLE);	//enables TIM14
	TIM_Cmd(LED_5_TIMER, ENABLE);	//enables TIM13
	

}

/******************************************************************************
* Description: <Initializes pwm for RGB light>
* Parameters:  <frequency; uint32_t; the number of times the pwm happens per 
*				second>
*			   <preScaler; uint16_t; Determines the speed of the clock for 
*				timer 11; the frequency of the timer will be the clock speed
*				of the main processor divided by the pre-scaler>
******************************************************************************/
void init_RGB_led_timers(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM4 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RGB_TIMER_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(RGB_BANK_CLOCK, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PD12 - PD14 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN | GREEN_LED_PIN | BLUE_LED_PIN;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(RGB_BANK, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(RGB_BANK, BLUE_LED_SOURCE_PIN, RGB_AF);
	GPIO_PinAFConfig(RGB_BANK, GREEN_LED_SOURCE_PIN, RGB_AF);
	GPIO_PinAFConfig(RGB_BANK, RED_LED_SOURCE_PIN, RGB_AF);
	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM4
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(RGB_TIMER, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM4 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(RGB_TIMER , &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	TIM_OC2Init(RGB_TIMER , &TIM_OCInitStructure);
	TIM_OC3Init(RGB_TIMER , &TIM_OCInitStructure);
	 
	// Enable TIM4 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(RGB_TIMER , TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(RGB_TIMER , TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(RGB_TIMER , TIM_OCPreload_Enable);
	 
	// Enable TIM4 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(RGB_TIMER , ENABLE);
	 
	// Enable TIM4 counter
	TIM_Cmd(RGB_TIMER , ENABLE); 
	
}

void init_muxes(void)
{
   /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/******************************************************************************
* Description: <Initializes the timer for the servo motors; period = 20ms>
* Parameters:  <none>
******************************************************************************/
void initialize_servo_timer(void)
{
	uint16_t frequency = 50;  //period of 20 ms
	uint16_t preScaler = 64;
	
	// Enable TIM3 and GPIOC clocks
	RCC_APB2PeriphClockCmd(SERVO_1_TIMER_CLOCK, ENABLE);
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

void initialize_stepper_pins()
{
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 

	GPIO_InitStructure.GPIO_Pin = STEPPER_HORIZONTAL_STEP_PIN | STEPPER_VERTICAL_STEP_PIN;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(STEPPER_STEP_BANK, &GPIO_InitStructure);	//initializes the structure
	
	GPIO_InitStructure.GPIO_Pin = STEPPER_HORIZONTAL_DIR_PIN | STEPPER_HORIZONTAL_ENABLE_PIN |
								  STEPPER_VERTICAL_DIR_PIN | STEPPER_VERTICAL_ENABLE_PIN;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(STEPPER_DIR_ENABLE_BANK, &GPIO_InitStructure);	//initializes the structure
}

void initialize_stepper_timer(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM12 and GPIOC clocks
	RCC_APB1PeriphClockCmd(STEPPER_TIMER_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(STEPPER_BANK_CLOCK, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = STEPPER_PIN1 | STEPPER_PIN2;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(STEPPER_BANK, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM12 functions.
	GPIO_PinAFConfig(STEPPER_BANK, STEPPER_SOURCE_PIN1, GPIO_AF_TIM12);
	GPIO_PinAFConfig(STEPPER_BANK, STEPPER_SOURCE_PIN2, GPIO_AF_TIM12);
	 
	// Compute prescaler value for timebase
	uint32_t PrescalerValue = (uint32_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do
	//second value in the divide is the frequency
	uint32_t PreCalPeriod = ((84000000 * preScaler) / frequency) - 1;  //To figure out what the numbers do

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM12
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the prescaller which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(STEPPER_TIMER, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM12 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(STEPPER_TIMER, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	TIM_OC2Init(STEPPER_TIMER, &TIM_OCInitStructure);
	
	 
	// Enable TIM12 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(STEPPER_TIMER, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(STEPPER_TIMER, TIM_OCPreload_Enable);
	
	// Enable TIM12 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(STEPPER_TIMER, ENABLE);
	 
	// Enable TIM12 counter
	TIM_Cmd(STEPPER_TIMER, ENABLE); 
	
	//return(PreCalPeriod);
}

void initialize_stepper_objects(void)
{
  horizontalStepper = Stepper_Initialize(
	STEPPER_STEP_BANK, STEPPER_HORIZONTAL_STEP_PIN,
	STEPPER_DIR_ENABLE_BANK, STEPPER_HORIZONTAL_DIR_PIN,
	STEPPER_DIR_ENABLE_BANK, STEPPER_HORIZONTAL_ENABLE_PIN,
	STEPPER_HORIZONTAL_POLARITY);
  
  verticalStepper = Stepper_Initialize(
	STEPPER_STEP_BANK, STEPPER_VERTICAL_STEP_PIN,
	STEPPER_DIR_ENABLE_BANK, STEPPER_VERTICAL_DIR_PIN,
	STEPPER_DIR_ENABLE_BANK, STEPPER_VERTICAL_ENABLE_PIN,
	STEPPER_VERTICAL_POLARITY);
}

void initialize_timer3(uint32_t frequency, uint16_t preScaler)
{
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(TURN_FOOT_BANK_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(BILGE_PUMP_BANK_CLOCK, ENABLE); 
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = TURN_FOOT_PIN1 | TURN_FOOT_PIN2;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(TURN_FOOT_BANK, &GPIO_InitStructure);	//initializes the structure
	 
	GPIO_InitStructure.GPIO_Pin = BILGE_PUMP_PIN1;  //specifies which pins are used
	GPIO_Init(BILGE_PUMP_BANK, &GPIO_InitStructure);	//initializes the structure
	
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(TURN_FOOT_BANK, TURN_FOOT_SOURCE_PIN1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(TURN_FOOT_BANK, TURN_FOOT_SOURCE_PIN2, GPIO_AF_TIM3);
	GPIO_PinAFConfig(BILGE_PUMP_BANK, BILGE_PUMP_SOURCE_PIN1, GPIO_AF_TIM3);
	 
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
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	 
	// Enable TIM3 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	 
	// Enable TIM3 counter
	TIM_Cmd(TIM3, ENABLE); 
	
}

void initialize_timer5(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
	
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //Frequency set to 1ms
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	 
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM5, ENABLE);
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//Initialized D5 as Tx and D6 as Rx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE); //Enable USART2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt

}

void init_UART5(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART6 initialization
	
	/* enable APB2 peripheral clock for USART5 
	 * note that only USART5 and USART6 are connected to APB1
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART5, PC12 for Tx and PD2 for Rx
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART5 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; 			// PC 12 (TX) 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baud rate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pull up resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; 				// PD2 (RX)
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART6 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART5 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				  // the baud rate is set to the value we passed into this function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;  // we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		  // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		  // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	
	USART_Init(UART5, &USART_InitStruct);					  // again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	USART_Cmd(UART5, ENABLE);	//Enables USART6
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // Enables Serial Interrupt
	
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
	
	GPIO_InitTypeDef GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART6 initialization

	
	/* enable APB2 peripheral clock for USART6 
	 * note that only USART6 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART6, PG9 for Rx and PG14 for TX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART6 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; // Pins 9 (RX) and 14 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baud rate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pull up resistors on the IO pins
	GPIO_Init(GPIOG, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART6 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	
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



