//This function works with a mega with the rx1 and tx1 conected together
//It reads the serial buffer and extracts the important information into a globally defined array

//Function protypes for the read function
boolean readPacket(void);
byte crc8(const byte *packet);
byte receivedCheckSum (const byte *packet);
boolean usePacket(void);

//Constants
#define STOP 0x01
#define CONTROL_MOTOR 0x02
#define REQUEST_FAULT_DATA 0x03
#define ADDRESS 0x01


//Variables to simulate an actual packet
byte startByte = 12;
byte address1 = 0x01;
byte command = 0x02;
byte Arg1 = 20;
byte Arg2 = 0;
byte checkSum = 0;
byte endByte = 13;

//Making the array a global array
byte receivedPacket[] = {
  1, 2, 3, 4, 5};

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);

  //Sends the simulated packet to Serial 1. 
  byte sendPacket[] = {startByte, address1, command, Arg1, Arg2, checkSum, endByte};
  sendPacket[5] = crc8(sendPacket);

  Serial1.write(sendPacket, sizeof(sendPacket));
  delay(1);
} 

void loop()
{  
  boolean Received = readPacket(); 
  if(Received)
  {
    usePacket();
  }
  delay(1000);
}

//The read function which return 0 if it is successful and a positive byte if there was an error. 
//The delays are in there in order to remove some noise whenever Serial.read() is happening 

boolean readPacket(void)
{
  //storage variables for the function
  byte byteReceived = 0;
  int index = 0;
  byte nextByte = 0; 

  if(Serial1.available())
  {
    //Checks if the first byte is a start byte.
    if(Serial1.read() == 12)
    {
      index = 0;
      //Reads the five important bytes into the array
      while(index < 5)
      {
        delay(1);

        //Makes sure there are no random 12 or 13 in the packet
        nextByte = Serial1.peek();
        if(nextByte == 12)
        { 
          return false;  //Start byte in wrong location
        }
        else if(nextByte == 13 && index != 4)
        {
          return false; //end byte in wrong location
        }
        delay(1);

        receivedPacket[index] = Serial1.read();
        index++;
      }
      delay(1);

      //Makes sure the last byte it a 13
      if(Serial1.read() != 13)
      {
        delay(1);
        return false; //last byte is an end byte
      }
      delay(1);

      byte receivedCheck = receivedCheckSum(receivedPacket);
      {
        //Function returns 0 if everything is successful 
        return true;  //Yay! Everything Works!
      }
      return false;  //check sum found an error

    }   

    //Retruns a number if the first byte is not a 12
    else
    {
      return false; //first byte of buffer is not start byte
    }
  }
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

byte receivedCheckSum(const byte *packet)
{ 

  byte crc = 0;
  for(byte len = 0; len < 4; len++) {
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

boolean usePacket(void)
{
 if(receivedPacket[0] == ADDRESS)
 {
   if(receivedPacket[1] == STOP)
   {
     digitalWrite(11, HIGH);
     return true;
   }
   else if(receivedPacket[1] == CONTROL_MOTOR)
   {
     analogWrite(12, receivedPacket[2]);
     analogWrite(10, receivedPacket[3]);
     return true;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     digitalWrite(10, HIGH);
     return true;
   }
   else
     return false;
 } 
 else
   return false;
}
