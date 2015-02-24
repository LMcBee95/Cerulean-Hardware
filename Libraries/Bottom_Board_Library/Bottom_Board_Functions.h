

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
