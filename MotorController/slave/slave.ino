// slave
byte message = 1;

//This function works with an attiny85 with the rx1 and tx1 conected together
//It reads the serial buffer and extracts the important information into a globally defined array

#include <SoftwareSerial.h>

//Function protypes for the read function
boolean readPacket(void);
byte crc8(const byte *packet);
byte receivedCheckSum (const byte *packet);
boolean usePacket(void);

byte arg1 = 0; 
byte sendByte = 1;

//Constants
#define STOP 0x02
#define CONTROL_MOTOR 0x02
#define REQUEST_FAULT_DATA 0x03
#define ADDRESS 0x02
#define RX 3
#define TX 4

//Declare Serial Object
SoftwareSerial mySerial(RX, TX);
//SoftwareSerial otherSerial(-1, 4);

//Making a global array
byte receivedPacket[] = {
  1, 2, 3, 4, 5};
  
byte newByte = 0x00;

void setup()
{
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  mySerial.begin(9600);
  //mySerial.begin(9600);
  digitalWrite(1, LOW);
  delay(2);
} 

void loop()
{  
  boolean received = readPacket(); 
  if(received)
  {  
    usePacket();
  }
}

//The read function which return 0 if it is successful and a positive byte if there was an error. 
//The delays are in there in order to remove some noise whenever Serial.read() is happening 

boolean readPacket(void)
{
  digitalWrite(1, LOW);
  
  //storage variables for the function
  byte byteReceived = 0;
  int index = 0;
  byte nextByte = 0; 
    
  if(mySerial.available())
  {
    //Dealy to compensate for the speed difference between the serial and how fast the attiny is going.
    //Checks if the first byte is a start byte.
    if(mySerial.read() == 0x12)
    {
      //Dealy to compensate for the speed difference between the serial and how fast the attiny is going.
      delay(5);
      index = 0;
      //Reads the five important bytes into the array
      while(index < 5)
      {
        //Makes sure there are no random 12 or 13 in the packet
        nextByte = mySerial.peek();
        if(nextByte == 0x12)
        { 
          digitalWrite(0, LOW);
          
          return false;  //Start byte in wrong location
        }
        else if(nextByte == 0x13 && index != 4)
        {
          digitalWrite(0, LOW);
          return false; //end byte in wrong location
        }

        receivedPacket[index] = mySerial.read();
        index++;
      }

      //Makes sure the last byte it a 13
      byteReceived = mySerial.read();
      if(byteReceived != 0x13)
      {
        digitalWrite(0, LOW);
        return false; //last byte is an end byte
      }

      byte receivedCheck = receivedCheckSum(receivedPacket);
      if(receivedPacket[4] == receivedCheck)
      {
        while(mySerial.available())
        {
         mySerial.read(); 
        }
        //Function returns true if everything is successful 
        return true;  //Yay! Everything Works!
      }
      digitalWrite(0, LOW);
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

  //Checks if this is the correct address
  //If the functions returns true then the device will do something. If it is false then the device will not do anything
  if(receivedPacket[0] == ADDRESS)
 {
   //This is where the specific commands are done.
   if(receivedPacket[1] == STOP)
   {
     return true;
   }
   else if(receivedPacket[1] == CONTROL_MOTOR)
   {
     return true;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     digitalWrite(0, HIGH);
     digitalWrite(1, HIGH);
     delay(1);
     mySerial.write(1);
     mySerial.write(2);
     mySerial.write(3);
     mySerial.write(4);
     mySerial.write(5);
     mySerial.write(6);
     sendByte++;
     delay(1);
     digitalWrite(1, LOW);
     return true;
   }
   else
     return false;
 } 
 else
 {
   digitalWrite(0, LOW);
   return false;
 }
}
