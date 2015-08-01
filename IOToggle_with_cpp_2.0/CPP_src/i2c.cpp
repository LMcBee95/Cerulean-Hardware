#include "i2c.h"


i2c::i2c(GPIO_TypeDef* sclBank, uint16_t scl_pin, GPIO_TypeDef* sdaBank, uint16_t sda_pin, I2C_TypeDef*  selected_I2C)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	I2Cx = selected_I2C;
	
	bankToClock(sclBank);  //sets the pins clock
	bankToClock(sdaBank);  //sets the pins clock
	
	i2c_bankToClcok(I2Cx);
	
	//initializes the scl pin
	GPIO_InitStruct.GPIO_Pin = scl_pin; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			
	GPIO_Init(sclBank, &GPIO_InitStruct);	

	GPIO_PinAFConfig(sclBank, scl_pin, getAlternateFunction(I2Cx));
	
	//initializes the sda pin
	GPIO_InitStruct.GPIO_Pin = sda_pin; 			
	GPIO_Init(sdaBank, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(sdaBank, sda_pin, getAlternateFunction(I2Cx));
	
	// configures I2C 
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2Cx, &I2C_InitStruct);				// init I2C
	
	// enable I2C1
	I2C_Cmd(I2Cx, ENABLE);
}

void i2c::write(uint64_t data, uint8_t address, bool writeMore = true)
{
	I2C_start(I2Cx, address, I2C_Direction_Transmitter);
	while(data)
	{
		I2C_send(I2C1, data & 0xFF); // writes the lower eight bits to the slave
		
		data >> 8;  //moves the next 8 bits into position
	}
	I2C_stop(I2Cx); // stop the transmission
	
}

uint8_t i2c::readAck(uint8_t address,  bool readMore = true)
{
	if(started  == false)
	{
		started = true;
		I2C_start(I2Cx, address, I2C_Direction_Receiver);
	}
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE); // enable acknowledge of recieved data
	
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	
	return I2C_ReceiveData(I2Cx); // read data from I2C data register and return data byte
		
}

uint8_t i2c::readNack(uint8_t address,  bool readMore = true)
{
	uint8_t data;

	if(started  == false)
	{
		started = true;
		I2C_start(I2Cx, address, I2C_Direction_Receiver);
	}
	
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE); // enable acknowledge of recieved data
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	
	return I2C_ReceiveData(I2Cx); // read data from I2C data register and return data byte
	
	
	
	started = false;
}


void i2c::I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	address = address << 1;
	
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void i2c::I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void i2c::I2C_send(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void i2c::i2c_bankToClcok(I2C_TypeDef*  I2Cx)
{
	if (I2Cx == I2C1)
	{
	    /* Enable I2C1 clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); 
	}
	else if (I2Cx == I2C2)
	{
	    /* Enable I2C2 clock */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); 
	}
	else if (I2Cx == I2C3)
	{
	      /* Enable I2C3 clcok */
	     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE); 
	}
}

uint8_t i2c::getAlternateFunction(I2C_TypeDef*  I2Cx)
{
	if (I2Cx == I2C1)
	{
	    return GPIO_AF_I2C1;
	}
	else if (I2Cx == I2C2)
	{
	    return GPIO_AF_I2C2;
	}
	else if (I2Cx == I2C3)
	{
	      return GPIO_AF_I2C3; 
	}
}

void i2c::bankToClock(GPIO_TypeDef* bank)
{
	
	//initializes the correct clock that goes with the inputed bank
	if (bank == GPIOA)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	}
	else if (bank == GPIOB)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	}
	else if (bank == GPIOC)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  	}
	else if (bank == GPIOD)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	}
	else if (bank == GPIOE)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	}
	else if (bank == GPIOF)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	}
	else if (bank == GPIOG)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	}
}

