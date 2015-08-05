#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f4xx_conf.h"

class serial
{

	public:
	
		/*
		* Function: constructor
		* Usage: serial mySerial(Rx Bank, Rx Pin, Tx Bank, Tx Bank, USART NUM, Bauderate, Yes(1)/No(0) Interrupt);
		* Where mySerial is the name of an instance of serial
		* ----------------------
		* Creates an instance of the serial class.
		*/
	
		serial(GPIO_TypeDef* bankPinRx, uint16_t pinRx, GPIO_TypeDef* bankPinTx, uint16_t pinTx, USART_TypeDef* USARTx, uint32_t baudrate, uint8_t interruptFlag = 0);
	
		void write(uint8_t* packet, uint8_t size); //Lets the user send a packet using USART
	
		uint8_t read();
	
	private:
	
		/* private instance variables */
		
		USART_TypeDef* thisUsart;
		
		/* private functions */
		void USART_puts(USART_TypeDef* USARTx, uint8_t data); //Sends data out using USART
		
		void usartToClock(USART_TypeDef* USARTx); // Initializes the clock for the given USART
		
		void bankToClock(GPIO_TypeDef* bank); //sets the correct clock for a certain bank
		
		uint16_t getPinSource(uint16_t pin); //Returns the pin source associated with a specific pin number
		
		uint8_t getUsartAltFunc(USART_TypeDef* USARTx); //Returns the correct alternate function mapping
};

#endif