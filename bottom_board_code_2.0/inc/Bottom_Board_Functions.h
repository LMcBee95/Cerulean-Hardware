#ifndef BOTTOM_BOARD_FUNCTIONS_H_
#define BOTTOM_BOARD_FUNCTIONS_H_

#define PACKET_SIZE 16
#define MOTOR_PACKET_SIZE 7
#define TOP_BOTTOM_BAUD 115200
#define BOTTOM_MOTOR_BAUD 57600

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

uint8_t checksum(uint8_t* packet, uint8_t size);

void convertTBtoBB(uint8_t* top);

void Delay(__IO uint32_t nCount);

uint8_t handleTopPacket(void);

void pollMotor(uint8_t address);

uint8_t readSlavePacket(void);

void resetMotor(uint8_t address);

void sendPackets(void);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_puts(USART_TypeDef* USARTx, uint8_t data);

//Initializations
void init_IRQ(void);

void init_LEDS(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_USART6(uint32_t baudrate);



#endif