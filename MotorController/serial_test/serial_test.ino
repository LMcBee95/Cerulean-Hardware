uint8_t topPacket[16];


uint8_t checksum(uint8_t* packet, uint8_t start_index, uint8_t size) {
	uint8_t crc = 0;
	*packet += start_index;
	for (uint8_t i = 1; i < 14; i++) {
		uint8_t inbyte = packet[i];
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0xD5;
			inbyte >>= 1;
		}
	}
	return crc;
}



void setup()
{
	
	
	//Puts mock data into the packet
	
	topPacket[1] = 80;
	topPacket[2] = 80;
	topPacket[3] = 80; 
	topPacket[4] = 80;
	topPacket[5] = 80;
	topPacket[6] = 80;
	topPacket[7] = 80;
	topPacket[8] = 80;
	topPacket[9] = 9;
	topPacket[10] = 10;
	topPacket[11] = 11;
	topPacket[12] = 12;
	topPacket[13] = 13;
	topPacket[14] = checksum(topPacket, 1, 13);
	topPacket[15] = 0x13;
	delay(10);
	topPacket[0] = 0x12;

	
	uint8_t state = 0;
	
	uint16_t timer = 0;

        Serial.begin(19200);
}

void sendPacket(void)
{
	for(uint8_t i = 0; i < 16; i++)
	{
	  Serial.write(topPacket[i]);
	}
}
	
	void loop()
	{
			sendPacket();
		        delay(200);
		
	}

