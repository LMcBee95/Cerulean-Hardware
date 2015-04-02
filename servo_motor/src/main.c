/*
 *	This program creates pulse width modulation on TIM(er) 3 to run a servo motor. Pins C6: channel 1.   
 *	The frequency is 50 Hz, which means the period is is 20 ms, the same rate that servos take commands.
 *	Servos take a pwm pulse between 1 and 2 ms. 1 ms being the servo turned to 0 degrees and 2 ms meaning 
 *  the servo is turned to 180 degrees. The servo must receive a command every 24 or so ms or else the servo
 *  motor will disengage. 
 */

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include <misc.h>			 
#include <stm32f4xx_tim.h>  //library that contains all of the code the makes the pwm work

#define MAXSERVO 2.1
#define MINSERVO 0.8
#define MAXSERVOANGLE 135.0


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

void USART_puts(USART_TypeDef* USARTx, uint8_t data){
		
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040)); 
		USART_SendData(USARTx, data);
}

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/*This function initializes the pins that are used in pwm, assigns them to their alternate functions, 
 *and then initializes the TIM(ers) for those pins. For parameters, it takes the frequency of the pwm 
 *and the pre scaler for the clock. The frequency will work fine for anything bellow 525000, but it is 
 *not guaranteed to work above that. The pre scaler divides into the stm boards own internal clock to 
 *get a clock speed for the timer. The pre scaler works good at 1, but it can take any multiple of two 
 *as its input. The function will return the period of the pwm which is used to calculate the duty cycle 
 *when setting the pwm for each pin. 
 *
 *When deciding on the prescaller, use lower numbers for when you need higher frequencies and use higher
 *numbers when you need lower frequencies.  
 */
uint16_t initialize_servo_timer(void)
{
	uint16_t frequency = 50;  //period of 20 ms
	uint16_t preScaler = 64;
	
	// Enable TIM3 and GPIOC clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;  //structure used by stm in initializing pins. 
	
	// Configure PC6-PC9 pins as AF, Pull-Down
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //specifies which pins are used
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//assigns the pins to use their alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	//initializes the structure
	 
	// Since each pin has multiple extra functions, this part of the code makes the alternate functions the TIM3 functions.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);

	 
	// Compute prescaler value for timebase
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / (84000000 / preScaler)) - 1;  //To figure out what the numbers do

	uint16_t PreCalPeriod = ((84000000 / preScaler) / frequency) - 1; 

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  //structure used by stm in initializing the pwm
    TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// Setup timebase for TIM3
	TIM_TimeBaseStructure.TIM_Period = PreCalPeriod;  //sets the period of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //sets the pre scaler which is divided into the cpu clock to get a clock speed that is small enough to use for timers
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //initializes this part of the code
	 
	// Initialize TIM3 for 4 channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //sets the time to be pulse width
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initiates this part of the pulse width modulation
	
	 
	// Enable TIM3 peripheral Preload register on CCR1 for 4 channels
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	 
	// Enable TIM3 peripheral Preload register on ARR.
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	 
	// Enable TIM3 counter
	TIM_Cmd(TIM3, ENABLE); 
	
	return(PreCalPeriod);
}

/*
 *This function controls the voltage outputted by each pwm pin. For parameters, it takes the TIM(er) of the 
 *pin that is being changed, which CCRx (Constant current reduction) should be selected, the period of the duty 
 *cycle (which can be obtained from the initialization function), and the desired duty cycle (between 0 and 255).
 */

void anologWrite(uint32_t channel, uint32_t period, uint8_t dutyCycle)
{
	channel = (period + 1) * dutyCycle / 255.0;
}

/*
 *	This function sets the angle of the servo motor. It takes the tim(er) and the second ardument is the channel 
 *  on that tim(er) that you want to move a servo on. The third argument is the angle you want to set the servo to.
 *  The servo varies from 0 to 180 degrees. The period of the servo will always be 50 Hz, and the variables MAX and 
 *  MIN should be adjusted to represented the pulse length in micro seconds that the pulse needs to be on for the servo
 *  to be at 0 degrees and 180 degrees respectively. 
 */

void setServoAngle(uint16_t channel, uint8_t angle, uint16_t period)
{ 	
	TIM3->CCR1 = ((period + 1) / 20) * ((MAXSERVO - MINSERVO) * angle / MAXSERVOANGLE + MINSERVO );
}


int main(void)
{
	uint16_t period = initialize_servo_timer(50, 64);
	
	//setAngle(TIM3->CCR1, 90, period);  //Sets the angle of the servo attached to pin C6 to 90 degrees
	
	uint32_t max = 2.25;  //max length of pulse in mili seconds that the servo will take
	uint32_t min = 0.75;  //min length of pulse in mili seconds that the servo will take
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  init_USART1(115200);
	
	while(1)
	{
		
		
<<<<<<< HEAD
=======
		setAngle(TIM3->CCR1, 0, period);
		
		
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		 
		Delay(0x1FFFFFF);
		
		
		setAngle(TIM3->CCR1, 70, period);
		
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		
		Delay(0x1FFFFFF);
		
		
		setAngle(TIM3->CCR1, 130, period);
		
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		
		Delay(0x1FFFFFF); 
	
>>>>>>> origin/master
	}
}