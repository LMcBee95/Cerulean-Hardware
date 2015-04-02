#define PACKET_SIZE 16
#define TOP_BOTTOM_BAUD 115200
#define BOTTOM_MOTOR_BAUD 57600

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC


extern uint8_t storage[PACKET_SIZE];	 //Array used to store the data received from the top board
extern uint8_t motor[8][7];	 //A multidimensional array to store all of the motor commands
extern uint8_t poll[7]; 		 //An array to store the packet that will poll the motors
extern volatile uint8_t pollReceived[7]; //An array used to store the packet received from the motors after they are polled
extern volatile uint8_t reset[7];		 //An array to send a reset command if one of the motors has a fault

extern volatile uint8_t counter;
extern volatile uint8_t pollCounter; //Keeps track of how many packets have been sent since we last polled a motor
extern volatile uint8_t pollAddress; //Stores the address of the motor that is going to be pulled next
extern volatile uint8_t received;


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
