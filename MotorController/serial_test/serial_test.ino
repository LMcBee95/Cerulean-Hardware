#define PACKET_SIZE 20

uint8_t topPacket[PACKET_SIZE];


uint8_t checksum(uint8_t* packet, uint8_t start_index, uint8_t size) {
  uint8_t crc = 0;
  *packet += start_index;
  for (uint8_t i = 1; i < PACKET_SIZE - 2; i++) {
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
	
        topPacket[16] = 1;
	topPacket[17] = 1;
	topPacket[1] = 10;
	topPacket[2] = 20;
	topPacket[3] = 30; 
	topPacket[4] = 40;
	topPacket[5] = 50;
	topPacket[6] = 60;
	topPacket[7] = 70;
	topPacket[8] = 80;
	topPacket[9] = 150;
	topPacket[10] = 100;
	topPacket[11] = 110;
	topPacket[12] = 120;
	topPacket[13] = 130;
        delay(10);
        topPacket[19] = 0x13;
        topPacket[14] = 140;
	topPacket[15] = 150;
        topPacket[18] = checksum(topPacket, 1, PACKET_SIZE - 3);
        topPacket[0] = 0x12;

	
	uint8_t state = 0;
	
	uint16_t timer = 0;

        Serial.begin(115200);
}

void sendPacket(void)
{
	for(uint8_t i = 0; i < PACKET_SIZE; i++)
	{
	  Serial.write(topPacket[i]);
	}
}
	
	void loop()
	{
<<<<<<< HEAD
	  sendPacket();
	  delay(1000);
=======
			sendPacket();
		        delay(200);
>>>>>>> origin/master
		
	}

