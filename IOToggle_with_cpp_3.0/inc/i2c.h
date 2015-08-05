#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx_conf.h"  //includes all of the peripheral libraries

class i2c
{
	public:
		
		/*
		* Function: constructor
		* Usage: gpio channel(Pin Bank, Pin Number, I2C channel);
		* 
		* ----------------------
		* Creates an instance of the i2c class.
		*/
	
		i2c(GPIO_TypeDef* sclBank, uint16_t scl_pin, GPIO_TypeDef* sdaBank, uint16_t sda_pin, I2C_TypeDef*  selected_I2C);
	
		/*
		* Function: write
		* Usage: write(57, true);
		* 
		* ----------------------
		* Writes one byte of data and determines whether to continue writingyou
	
		*/
	
		void write(uint64_t data, uint8_t address, bool writeMore);
	
		/*
		* Function: read
		* Usage: read(address, true);
		* 
		* ----------------------
		* Reads in one byte of data and determines whether to continue reading
		*/
		
		uint8_t readAck(uint8_t address, bool readMore);
		
		/*
		* Function: readNack
		* Usage: readNack(true);
		* 
		* ----------------------
		* Reads in one byte of data and determines whether to continue reading
		*/
		
		uint8_t readNack(uint8_t address, bool readMore);
	
	
	private:
		
	I2C_TypeDef* I2Cx;  //stores the i2c channel being used
	bool started = false;  //variable used to store whether the transmition has been started
	
	
	void bankToClock(GPIO_TypeDef* bank); //sets the correct clock bassed on the inputed bank
	void i2c_bankToClcok(I2C_TypeDef*  selected_I2C); //sets the correct clock based on the i2c
	uint8_t getAlternateFunction(I2C_TypeDef*  I2Cx);
	void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
	void I2C_stop(I2C_TypeDef* I2Cx);
	void I2C_send(I2C_TypeDef* I2Cx, uint8_t data); //sends data via I2C
};


#endif
