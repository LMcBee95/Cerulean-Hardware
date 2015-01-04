#include "functions.h"

//Usart6: PC7, PC6 - Motor bus also need PC8, PC9

//Output for 1, AF output for 2, input for else


/*************************** X5 Code ********************************************/
void PinOutput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK){

  RCC_AHB1PeriphClockCmd(CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(PORT, &GPIO_InitStructure);
}

void PinInput(const uint16_t PIN, GPIO_TypeDef* PORT,const uint32_t CLK){

  RCC_AHB1PeriphClockCmd(CLK, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(PORT, &GPIO_InitStructure);
}

void Mux_Init(){
  PinOutput(MUX_0_PIN, MUX_0_PORT, MUX_0_CLK);
  GPIO_ResetBits(MUX_0_PORT, MUX_0_PIN);
  PinOutput(MUX_1_PIN, MUX_1_PORT, MUX_1_CLK);
  GPIO_ResetBits(MUX_1_PORT, MUX_1_PIN);
}

void TurnerSelectInit(){
  PinOutput(TURNER_SEL_0_PIN, TURNER_SEL_0_PORT, TURNER_SEL_0_CLK);
  GPIO_ResetBits(TURNER_SEL_0_PORT, TURNER_SEL_0_PIN);
  PinOutput(TURNER_SEL_1_PIN, TURNER_SEL_1_PORT, TURNER_SEL_1_CLK);
  GPIO_ResetBits(TURNER_SEL_1_PORT, TURNER_SEL_1_PIN);
}

void TurnerDirInit(){
  PinOutput(TURNER_DIR_PIN, TURNER_DIR_PORT, TURNER_DIR_CLK);
}

void WaterInit(){
  PinInput(WATER_0_PIN, WATER_0_PORT, WATER_0_CLK);
  PinInput(WATER_1_PIN, WATER_1_PORT, WATER_1_CLK);
  PinInput(WATER_2_PIN, WATER_2_PORT, WATER_2_CLK);
  PinInput(WATER_3_PIN, WATER_3_PORT, WATER_3_CLK);
}

void FuseInit(){
  PinInput(FUSE_DECT_PIN, FUSE_DECT_PORT, FUSE_DECT_CLK);
}

void TurnerPWMInit(){
  /* GPIOD clock enable */
  RCC_APB2PeriphClockCmd(TURNER_PWM_TIM_RCC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = TURNER_PWM_1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(TURNER_PWM_1_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(TURNER_PWM_1_PORT, TURNER_PWM_1_SOURCE, TURNER_PWM_TIM_AF);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TURNER_PWM_TIM, &TIM_TimeBaseStructure);

  /* Enable TIMx Preload register on ARR */
  TIM_ARRPreloadConfig(TURNER_PWM_TIM, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TURNER_PWM_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TURNER_PWM_TIM, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TURNER_PWM_TIM, ENABLE);
  TURNER_PWM_TIM->CCR1 = 0;                                                      
}

void NVIC_Config(void){

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void LedInit(){
  PinOutput(LED_0_PIN, LED_0_PORT, LED_0_CLK);
  PinOutput(LED_1_PIN, LED_1_PORT, LED_1_CLK);
  PinOutput(LED_2_PIN, LED_2_PORT, LED_2_CLK);
  PinOutput(LED_3_PIN, LED_3_PORT, LED_3_CLK);
}

void Usart2Init(){ 

  //BufferInit(&U2Rx);
  //BufferInit(&U2Tx);

//  NVIC_Config();
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
 
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART2, &USART_InitStructure);
  
  USART_Cmd(USART2, ENABLE);
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

void Usart6Init(){
  PinOutput(USART6_ENABLE_PIN, USART6_ENABLE_PORT, USART6_ENABLE_CLK);
  PinOutput(USART6_DISABLE_PIN, USART6_DISABLE_PORT, USART6_DISABLE_CLK);
  
  BufferInit(&U6Rx);
  BufferInit(&U6Tx);

  NVIC_Config();
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
  
  USART_InitTypeDef USART_InitStructure;
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART6, &USART_InitStructure);
  
  USART_Cmd(USART6, ENABLE);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
}

void MUXControl(volatile uint8_t M0, volatile uint8_t M1){
  if(M0 == 1){
    GPIO_SetBits(MUX_0_PORT, MUX_0_PIN);
  }else{
    GPIO_ResetBits(MUX_0_PORT, MUX_0_PIN);
  }

  if(M1 == 1){
    GPIO_SetBits(MUX_1_PORT, MUX_1_PIN);
  }else{
    GPIO_ResetBits(MUX_1_PORT, MUX_1_PIN);
  }
}

void delay(int i){    //100 000 is about 1/4th a second, 100 000 000 is about 2.5 sec
  while (i-- > 0) {
    asm("nop");       /* This stops it optimising code out */
  }
}

uint16_t GenChecksum(uint16_t address, uint16_t command){
  return (address+command)&0xFF;
}

void ResetMotorAddress(){

  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN); 
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //Set 0
  USART_SendData(USART6, 0x30);//0
  delay(3000);
  
  USART_SendData(USART6, 0x30);//0
  delay(3000);

  //Set 0
  USART_SendData(USART6, 0x30);//0
  delay(3000);
  
  USART_SendData(USART6, 0x30);//0
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
 
 GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
 GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}

void UpdateAddress(uint16_t OldAddress, uint16_t NewAddress){
  //Getting things set up  
  int OldAddress1 = ((OldAddress & 0xF0) >> 4);
  if(OldAddress1 >= 0x0A){
      OldAddress1 += 0x37;
  }else{
      OldAddress1 += 0x30;
  }

  int OldAddress2 = ((OldAddress & 0x0F)) ;
  if(OldAddress2 >= 0x0A){
      OldAddress2 += 0x37;
  }else{
      OldAddress2 += 0x30;
  } 
  
  int NewAddress1 = ((NewAddress & 0xF0) >> 4);
  if(NewAddress1 >= 0x0A){
      NewAddress1 += 0x37;
  }else{
      NewAddress1 += 0x30;
  }

  int NewAddress2 = ((NewAddress & 0x0F)) ;
  if(NewAddress2 >= 0x0A){
      NewAddress2 += 0x37;
  }else{
      NewAddress2 += 0x30;
  }  
  
  int Checksum = (NewAddress+OldAddress+0x0B)&0xFF;
  int Checksum1 = ((Checksum & 0xF0) >> 4);
  if(Checksum1 >= 0x0A){
      Checksum1 += 0x37;
  }else{
      Checksum1 += 0x30;
  }

  int Checksum2 = ((Checksum & 0x0F));
  if(Checksum2 >= 0x0A){
      Checksum2 += 0x37;
  }else{
      Checksum2 += 0x30;
  }

  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //Old address byte
  USART_SendData(USART6, OldAddress1);
  delay(3000);
  
  USART_SendData(USART6, OldAddress2);
  delay(3000);

  //Change address command
  USART_SendData(USART6, 0x30);//0
  delay(3000);
  
  USART_SendData(USART6, 0x42);//B
  delay(3000);
  
  //New Address byte
  USART_SendData(USART6, NewAddress1);
  delay(3000);
  
  USART_SendData(USART6, NewAddress2);
  delay(3000);
  
  //checksum byte - Same as new address
  USART_SendData(USART6, Checksum1);
  delay(3000);
  
  USART_SendData(USART6, Checksum2);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}

void MotorControlRS485(uint16_t Address, uint16_t Speed){
  //Getting things set up  
  int Address1 = ((Address & 0xF0) >> 4);
  if(Address1 >= 0x0A){
      Address1 += 0x37;
  }else{
      Address1 += 0x30;
  }

  int Address2 = ((Address & 0x0F)) ;
  if(Address2 >= 0x0A){
      Address2 += 0x37;
  }else{
      Address2 += 0x30;
  }  
  
  int Speed1 = ((Speed & 0xF0) >> 4);
  if(Speed1 >= 0x0A){
      Speed1 += 0x37;
  }else{
      Speed1 += 0x30;
  }

  int Speed2 = ((Speed & 0x0F));
  if(Speed2 >= 0x0A){
      Speed2 += 0x37;
  }else{
      Speed2 += 0x30;
  }

  int Checksum = (Address+Speed)&0xFF;
  int Checksum1 = ((Checksum & 0xF0) >> 4);
  if(Checksum1 >= 0x0A){
      Checksum1 += 0x37;
  }else{
      Checksum1 += 0x30;
  }

  int Checksum2 = ((Checksum & 0x0F));
  if(Checksum2 >= 0x0A){
      Checksum2 += 0x37;
  }else{
      Checksum2 += 0x30;
  }
  
  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //address byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
  
  //speed byte
  USART_SendData(USART6, Speed1);
  delay(3000);
  
  USART_SendData(USART6, Speed2);
  delay(3000);
  
  //nothing byte
  USART_SendData(USART6, 0x30);
  delay(3000);
  
  USART_SendData(USART6, 0x30);
  delay(3000);
     
  //checksum byte
  USART_SendData(USART6, Checksum1);
  delay(3000);
  
  USART_SendData(USART6, Checksum2);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}


void RequestStatus(uint16_t Address){
  //Getting things set up  
  int Address1 = ((Address & 0xF0) >> 4);
  if(Address1 >= 0x0A){
      Address1 += 0x37;
  }else{
      Address1 += 0x30;
  }

  int Address2 = ((Address & 0x0F)) ;
  if(Address2 >= 0x0A){
      Address2 += 0x37;
  }else{
      Address2 += 0x30;
  }  
  
  GPIO_SetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_SetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);  
  
  //Start !
  USART_SendData(USART6, 0x24);
  delay(3000);
     
  //address byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
     
  //checksum byte
  USART_SendData(USART6, Address1);
  delay(3000);
  
  USART_SendData(USART6, Address2);
  delay(3000);
    
  //end $
  USART_SendData(USART6, 0x21);
  delay(3000);
  
  GPIO_ResetBits(USART6_ENABLE_PORT, USART6_ENABLE_PIN);
  GPIO_ResetBits(USART6_DISABLE_PORT, USART6_DISABLE_PIN);
}

void ShiftInit(){
  PinOutput(SHIFT_MOSI_PIN, SHIFT_MOSI_PORT, SHIFT_MOSI_CLK);
  PinOutput(SHIFT_CLK_PIN, SHIFT_CLK_PORT, SHIFT_CLK_CLK);
  PinOutput(SHIFT_LE_PIN, SHIFT_LE_PORT, SHIFT_LE_CLK);
  PinOutput(SHIFT_OE_PIN, SHIFT_OE_PORT, SHIFT_OE_CLK);

  //GPIO_SetBits(SHIFT_OE_PORT, SHIFT_OE_PIN);
  GPIO_ResetBits(SHIFT_OE_PORT, SHIFT_OE_PIN);
  GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
  GPIO_ResetBits(SHIFT_LE_PORT, SHIFT_LE_PIN);
  GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);

  //ShiftControl(0,0,1,0);
}

void ShiftControl( volatile uint8_t manip, volatile uint8_t tool2, volatile uint8_t tool3, volatile uint8_t tool4){
  
  uint16_t d = 10;
  //printf("%d %d %d %d\n", manip, tape, patch, lift);
  //USART_SendData(USART3, manip);
  //USART_SendData(USART3, tape);
  //USART_SendData(USART3, patch);
  //USART_SendData(USART3, lift);
  //USART_SendData(USART3, 0x91);
  
  //1 and 2 - manip
  //3 and 4 - tool2
  //4 and 5 - tool3
  //6 and 7 - tool4
  
  GPIO_SetBits(SHIFT_LE_PORT, SHIFT_LE_PIN);

if(manip == 0){
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
}else{
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
   
    GPIO_SetBits(LED_0_PORT, LED_0_PIN); 
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
}   

  if(manip == 0){
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(LED_0_PORT, LED_0_PIN); 
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }

  if(tool2 == 0){
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
     GPIO_SetBits(LED_1_PORT, LED_1_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }
  
  if(tool2 == 0){
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    GPIO_SetBits(LED_1_PORT, LED_1_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }

if(tool3 == 0){
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
     GPIO_SetBits(LED_2_PORT, LED_2_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }
  
  
  if(tool3 == 0){
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
     GPIO_SetBits(LED_2_PORT, LED_2_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  } 

  if(tool4 == 0){
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
     GPIO_SetBits(LED_3_PORT, LED_3_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  } 
  
  if(tool4 == 0){
    GPIO_SetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
    
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  }else{
    GPIO_ResetBits(SHIFT_MOSI_PORT, SHIFT_MOSI_PIN);
    delay(d);
     GPIO_SetBits(LED_3_PORT, LED_3_PIN); 
    GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
  } 
    /*GPIO_SetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
    delay(d);
    GPIO_ResetBits(SHIFT_CLK_PORT, SHIFT_CLK_PIN);
*/
    delay(d);

   // GPIO_ResetBits(SHIFT_OE_PORT, SHIFT_OE_PIN);
    GPIO_ResetBits(SHIFT_LE_PORT, SHIFT_LE_PIN);

    delay(1000000);
    //GPIO_SetBits(SHIFT_OE_PORT, SHIFT_OE_PIN);
}


void MotorSet(int Motor, int Dir, int Speed){

  //Motor1
  if(Motor == 1){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    }
    TIM4->CCR1 = Speed;
  }
  
  //Motor2
  if(Motor == 2){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN);
    }
    else{
      //Backward
      GPIO_ResetBits(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN);
    }
    TIM4->CCR2 = Speed;
  }
  
  //Motor 3
  if(Motor == 3){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN);
    }
    TIM4->CCR3 = Speed;
  }
  
  //Motor 4
  if(Motor == 4){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN);
    }
    TIM4->CCR4 = Speed;
  }
  
  //Motor 5
  if(Motor == 5){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN);
    }
    TIM3->CCR1 = Speed;
  }
  
  //Motor 6
  if(Motor == 6){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN);
    }
    TIM3->CCR2 = Speed;
  }
  
  //Motor 7
  if(Motor == 7){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR7_DIR_PORT, MOTOR7_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR7_DIR_PORT, MOTOR7_DIR_PIN);
    }
    TIM3->CCR3 = Speed;
  }
  
  //Motor 8
  if(Motor == 8){
    if (Dir ==1){
      //forward
      GPIO_SetBits(MOTOR8_DIR_PORT, MOTOR8_DIR_PIN);
    }
    else{
        //reverse
      GPIO_ResetBits(MOTOR8_DIR_PORT, MOTOR8_DIR_PIN);
    }
    TIM3->CCR4 = Speed;
  }
  
}

void MotorControl( volatile uint8_t M1, volatile uint8_t M2, volatile uint8_t M3, volatile uint8_t M4, volatile uint8_t M5, volatile uint8_t M6, volatile uint8_t M7, volatile uint8_t M8){

  uint16_t period =   10000 / 2000;

  
  //-100 = 156
  //-25 = 231
  
  if(M1<=100){
    MotorSet(1, 0, period * M1 );
  }else if(M1>=156){
    MotorSet(1, 1, period * ( 100 - (M1 -156)));
  }
  
  if(M2<=100){
    MotorSet(2, 0, period * M2 );
  }else if(M2>=156){
    MotorSet(2, 1, period * ( 100 - (M2 -156)));
  }
  
  if(M3<=100){
    MotorSet(3, 0, period * M3);
  }else if(M3>=156){
    MotorSet(3, 1, period * ( 100 - (M3 -156)));
  }
  
  if(M4<=100){
    MotorSet(4, 0, period * M4);
  }else if(M4>=156){
    MotorSet(4, 1, period * ( 100 - (M4 -156)));
  }
  
  if(M5<=100){
    MotorSet(5, 0, period * M5);
  }else if(M5>=156){
    MotorSet(5, 1, period * ( 100 - (M5 -156)));
  }
  
  if(M6<=100){
    MotorSet(6, 0, period * M6);
  }else if(M6>=156){
    MotorSet(6, 1, period * ( 100 - (M6 -156)));
  }
  
  if(M7<=100){
    MotorSet(7, 0, period * M7);
  }else if(M7>=156){
    MotorSet(7, 1, period * ( 100 - (M7 -156)));
  }
  
  if(M8<=100){
    MotorSet(8, 0, period * M8);
  }else if(M8>=156){
    MotorSet(8, 1, period * ( 100 - (M8 -156)));
  }
  
}

void Motor1Init(){
  //Direction Pin
  PinOutput(MOTOR1_DIR_PIN, MOTOR1_DIR_PORT, MOTOR1_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR1_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR1_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR1_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR1_PWM_PORT, GPIO_PinSource12, GPIO_AF_TIM4);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM4, ENABLE);
  MotorSet(1,1,0);
}

void Motor2Init(){
  //Direction Pin
  PinOutput(MOTOR2_DIR_PIN, MOTOR2_DIR_PORT, MOTOR2_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR2_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR2_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR2_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR2_PWM_PORT, GPIO_PinSource13, GPIO_AF_TIM4);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM4, ENABLE);
  MotorSet(2,1,0);
}

void Motor3Init(){
  //Direction Pin
  PinOutput(MOTOR3_DIR_PIN, MOTOR3_DIR_PORT, MOTOR3_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR3_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR3_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR3_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR3_PWM_PORT, GPIO_PinSource14, GPIO_AF_TIM4);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM4, ENABLE);
  MotorSet(3,1,0);
}

void Motor4Init(){
  //Direction Pin
  PinOutput(MOTOR4_DIR_PIN, MOTOR4_DIR_PORT, MOTOR4_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR4_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR4_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR4_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR4_PWM_PORT, GPIO_PinSource15, GPIO_AF_TIM4);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM4, ENABLE);
  MotorSet(4,1,0);
}



void Motor5Init(){
  //Direction Pin
  PinOutput(MOTOR5_DIR_PIN, MOTOR5_DIR_PORT, MOTOR5_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR5_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR5_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR5_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR5_PWM_PORT, GPIO_PinSource4, GPIO_AF_TIM3);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM3, ENABLE);
  MotorSet(5,1,0);
}

void Motor6Init(){
  //Direction Pin
  PinOutput(MOTOR6_DIR_PIN, MOTOR6_DIR_PORT, MOTOR6_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR6_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR6_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR6_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR6_PWM_PORT, GPIO_PinSource5, GPIO_AF_TIM3);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM3, ENABLE);
  MotorSet(6,1,0);
}

void Motor7Init(){
  //Direction Pin
  PinOutput(MOTOR7_DIR_PIN, MOTOR7_DIR_PORT, MOTOR7_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR7_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR7_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR7_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR7_PWM_PORT, GPIO_PinSource0, GPIO_AF_TIM3);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM3, ENABLE);
  MotorSet(7,1,0);
}

void Motor8Init(){
  //Direction Pin
  PinOutput(MOTOR8_DIR_PIN, MOTOR8_DIR_PORT, MOTOR8_DIR_CLK);

  /* --------------------------- System Clocks Configuration -----------------*/
  /* TIMx clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); 

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(MOTOR8_PWM_CLK , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = MOTOR8_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(MOTOR8_PWM_PORT, &GPIO_InitStructure);

  /* Connect TIMx pins to AF */
  GPIO_PinAFConfig(MOTOR8_PWM_PORT, GPIO_PinSource1, GPIO_AF_TIM3);//need pinsource not just pin

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t Period;

  Period = 1000000 / 2000; // 20 KHz for 1MHz prescaled
       //100000
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 10000000) / 2) - 1; // Get clock to 1 MHz on STM32F4
  TIM_TimeBaseStructure.TIM_Period = Period - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Enable TIMx Preload register on ARR */

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM PWMXZ Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Period / 2; // 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* TIMx enable counter */
  TIM_Cmd(TIM3, ENABLE);
  MotorSet(8,1,0);
}


/****************** Usart & Buffer Functions ******************/
void Usart2Put(uint8_t ch){
  //put char to the buffer
  BufferPut(&U2Tx, ch);
  //enable Transmit Data Register empty interrupt
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

uint8_t Usart2Get(void){
  uint8_t ch;
  //check if buffer is empty
  while (BufferIsEmpty(U2Rx) ==SUCCESS);
  BufferGet(&U2Rx, &ch);
  return ch;
}

void Usart6Put(uint8_t ch){
  //put char to the buffer
  BufferPut(&U6Tx, ch);
  //enable Transmit Data Register empty interrupt
  USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
}

uint8_t Usart6Get(void){
  uint8_t ch;
  //check if buffer is empty
  while (BufferIsEmpty(U6Rx) ==SUCCESS);
  BufferGet(&U6Rx, &ch);
  return ch;
}

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
void BufferInit(__IO FIFO_TypeDef *buffer){

  buffer->count = 0;//0 bytes in buffer
  buffer->in = 0;//index points to start
  buffer->out = 0;//index points to start
}

ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch){
  if ( ch == 0x0C){
    buffer->in=0;
  }
  //if(buffer->count==USARTBUFFSIZE)
  //  return ERROR;//buffer full
  buffer->buff[buffer->in++]=ch;
  //buffer->count++;
  if(buffer->in==USARTBUFFSIZE)
    buffer->in=0;//start from beginning
  return SUCCESS;
}

ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch){
  if(buffer->count==0)
    return ERROR;//buffer empty
  *ch=buffer->buff[buffer->out++];
  buffer->count--;
  if(buffer->out==USARTBUFFSIZE)
    buffer->out=0;//start from beginning
  return SUCCESS;
}

ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer){
  if(buffer.count==0)
    return SUCCESS;//buffer full
  return ERROR;
}


