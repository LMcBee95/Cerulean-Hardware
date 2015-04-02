#define PACKET_SIZE 16
#define TOP_BOTTOM_BAUD 115200
#define BOTTOM_MOTOR_BAUD 57600

#define USART6_ENABLE_PIN			GPIO_Pin_8					//check if these are correct 
#define USART6_ENABLE_PORT			GPIOC
#define USART6_ENABLE_CLK			RCC_AHB1Periph_GPIOC

#define USART6_DISABLE_PIN			GPIO_Pin_9					//check if these are correct 
#define USART6_DISABLE_PORT			GPIOC
#define USART6_DISABLE_CLK			RCC_AHB1Periph_GPIOC

void init_IRQ(void);

void init_LEDS(void);

void init_USART1(uint32_t baudrate);

void init_USART2(uint32_t baudrate);

void init_USART6(uint32_t baudrate);

