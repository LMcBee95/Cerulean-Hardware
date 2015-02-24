#include <SoftwareSerial.h>

#define BAUD_RATE 57600
#define START_BYTE 0x12
#define END_BYTE  0x13
#define ADDRESS 8
#define SPEED 127
#define DIRECTION 1
#define MOTOR_COMMAND 1

#define RX 6
#define TX 7

byte packet[7] = {START_BYTE, ADDRESS, MOTOR_COMMAND, SPEED, DIRECTION, 0, END_BYTE};

byte crc8(const byte *packet);

SoftwareSerial mySerial(RX, TX);

void setup()
{
  mySerial.begin(BAUD_RATE);
  
  packet[5] = crc8(packet, 1, 4);
  
}

void loop()
{
  for(int i = 0; i < 7; i++)
  {
   mySerial.write(packet[i]); 
  }
  delay(1000);
}

byte crc8(const byte *packet, const byte pos_first_byte, const byte num_bytes_data)
{ 

  byte crc = 0;
  for(byte len = pos_first_byte; len < (pos_first_byte + num_bytes_data); len++) {
    uint8_t inbyte = *packet++;
    for (byte i = 8; i; i--) {
      byte mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0xD5;
      inbyte >>= 1;
    }
  }
  return crc;
}
