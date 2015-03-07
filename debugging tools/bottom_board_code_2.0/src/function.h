

/Creates a check sum 
//You pass in the name of the array, and how many bytes you want the checksum to read
uint8_t checksum(uint8_t* packet, uint8_t packetSize);

/*
 *	The function then converts the information received from the top packet into values
 *  that the bottom board can use. It takes the name of the packet that the packet from 
 *  the top board was stored in as it's argument.
 */
void Delay(__IO uint32_t nCount);

/*
 *	The function then converts the information received from the top packet into values
 *  that the bottom board can use. It takes the name of the packet that the packet from 
 *  the top board was stored in as it's argument.
 */
void convertTBtoBB(uint8_t* top);

/*
 *	This function handles receiving the top packet and then sending out the commands to the motor  
 *	controllers. It takes no arguments. The function will keep reading data until it receives a 0x12. Then the
 *	function stores the information sent from the top board into an array, produces motor control commands from the 
 *	top packet,and then sends the commands to all of the motors. The function checks before it sends 
 * 	the command to make sure the check sum from the top packet is correct and that the last byte is a 0x13. If this is 
 * 	true,the function returns a one and if it isn't,the function returns a 0.
 */
uint8_t handleTopPacket(void);

/*
 *	This function initializes all of the leds and the read write enabler pin.
 *	It initializes the the clock and then sets the pins as outputs. 
 */
void init_pins(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void pollMotor(uint8_t address);

uint8_t readSlavePacket(void);

void resetMotor(uint8_t address);

void sendPackets(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);