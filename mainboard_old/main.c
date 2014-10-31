#include "functions.h"
volatile FIFO_TypeDef U2Rx, U2Tx, U6Rx, U6Tx;
volatile uint8_t flag=0;


int main(void){
  
  SystemInit();
  //ShiftInit();
  //LedInit();
  Usart6Init();
  Usart2Init();
  Motor1Init();
  Motor2Init();
  Motor3Init();
  Motor4Init();
  Motor5Init();
  Motor6Init();
  Motor7Init();
  Motor8Init();
  //MotorSet(1,1,0); 
  //MotorSet(2,1,20);
  //MotorSet(3,1,30);
  //MotorSet(4,1,40);
  //MotorSet(5,1,100);
  //MotorSet(6,1,200);
  //MotorSet(7,1,70);
  //MotorSet(8,1,700);
  //MotorSet(1,1,50);
  //GPIO_SetBits(LED_0_PORT, LED_0_PIN); 
  while(1){
      MotorControl(U2Rx.buff[1],U2Rx.buff[2],U2Rx.buff[3],U2Rx.buff[4],U2Rx.buff[5],U2Rx.buff[6],U2Rx.buff[7],U2Rx.buff[8]);
      //MotorSet(1,1,1); 
      ShiftControl(U2Rx.buff[9], U2Rx.buff[10],U2Rx.buff[11], U2Rx.buff[12]);
     // ShiftControl(1,0,1,1);
     // GPIO_SetBits(LED_3_PORT, LED_3_PIN); 
  }
  
}
 
                
                
//CODE TO UPDATE ADDRESS                
/*
   GPIO_SetBits(LED_2_PORT, LED_2_PIN); 
   delay(33333333);
   UpdateAddress(0x55, 0x54);
   GPIO_ResetBits(LED_2_PORT, LED_2_PIN);
   GPIO_SetBits(LED_3_PORT, LED_3_PIN);
*/
  
//CODE TO RESET MOTOR
/*                
   GPIO_SetBits(LED_0_PORT, LED_0_PIN); 
   delay(33333333);
   ResetMotorAddress();
   GPIO_ResetBits(LED_0_PORT, LED_0_PIN);
   GPIO_SetBits(LED_2_PORT, LED_2_PIN);      
*/               
      
//CODE TO REQUEST STATUS AND SPIN MOTOR
///*
    //delay(33333333);
    //MotorControl(0x51, 0x80);
    
      
      
   /*
            RequestStatus(0x51);
            RequestStatus(0x55);
            MotorControl(0x51, 0xB4);
            MotorControl(0x55, 0xB4);            
            GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
 	    GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
            GPIO_SetBits(LED_0_PORT, LED_0_PIN);
            delay(33333333);
            GPIO_ResetBits(LED_0_PORT, LED_0_PIN);
            GPIO_SetBits(LED_2_PORT, LED_2_PIN);
            MotorControl(0x51, 0x80);
            MotorControl(0x55, 0x80);
            delay(33333333);
            GPIO_ResetBits(LED_2_PORT, LED_2_PIN);
      */
      
	
//*/    

/*	while(1){
          


		GPIO_SetBits(LED_1_PORT, LED_1_PIN);
                //USART_SendData(USART6, 0x12);
                delay(10000000);
                GPIO_ResetBits(LED_1_PORT, LED_1_PIN);
               // USART_SendData(USART6, 0x13);
                delay(10000000);
                
                
		/

		//LF, RF, LB, RB, Nothing , RFV, LBV, RBV, nothing, LFV
		MotorControl(U3Rx.buff[1],U3Rx.buff[2],U3Rx.buff[3],U3Rx.buff[4],0,U3Rx.buff[6],U3Rx.buff[7],U3Rx.buff[8],0,U3Rx.buff[5]);
		ShiftControl(U3Rx.buff[9], U3Rx.buff[10],U3Rx.buff[11], U3Rx.buff[12]);
		MUXControl(U3Rx.buff[13],U3Rx.buff[14]);
		LedControl(U3Rx.buff[15],U3Rx.buff[16],U3Rx.buff[17]);
	
		//MotorControl(0,0,0,100,0,0,0,0,0,0);
		
		//use usart, not buffer!
		
		USART_SendData(USART3, 0x12);
		delay(170000);
	
		//send data back here!

		if(GPIO_ReadInputDataBit(INDUCTION_PORT, INDUCTION_PIN) == (uint8_t)Bit_SET){
			USART_SendData(USART3, 0x30);
			delay(170000);
		}else{
			USART_SendData(USART3, 0x31);
			delay(170000);
		}
		USART_SendData(USART3, 0x13);
		delay(170000);
                /
	}
*/

