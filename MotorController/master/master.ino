// Master

//This function works with a mega with the rx1 and tx1 conected together
//It reads the serial buffer and extracts the important information into a globally defined array

byte crc8(const byte *packet);

//Constants
#define STOP 0x01
#define CONTROL_MOTOR 0x02
#define REQUEST_FAULT_DATA 0x03
#define ADDRESS1 0x01
#define ADDRESS2 0x02


//Variables to simulate an actual packet
byte startByte = 0x12;
byte address1 = 0x01;
byte command = 0x03;//STOP;
byte Arg1 = 0x18;
byte Arg2 = 0x04;
byte checkSum = 0xFF;
byte endByte = 0x13;

byte sendPacket[] = {startByte, address1, command, Arg1, Arg2, checkSum, endByte};

int counter = 0;

void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial2.begin(9600);
  pinMode(3, OUTPUT);
  
  digitalWrite(3, HIGH);
  delay(2);
 } 

void loop()
{ 
  if(counter % 2 == 0)
  { 
    sendPacket[1] = ADDRESS2;
    counter++;
  }
  else if(counter % 2 == 1)
  {
   sendPacket[1] = ADDRESS1;
   counter++; 
  }
  sendPacket[5] = crc8(sendPacket);
  for(int i = 0; i < 7; i++)
  {
     Serial3.write(sendPacket[i]); 
     Serial3.flush();
  }
  delay(1);
  digitalWrite(3, LOW);
  
  delay(7);
  
  while(Serial3.available())
  {
   byte receivedByte = Serial3.read();
   Serial.println(receivedByte); 
  }
  digitalWrite(3, HIGH);
  delay(1);
}

byte crc8(const byte *packet)
{ 
  byte crc = 0;
  *packet++;
  for(byte len = 1; len < 5; len++) {
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
