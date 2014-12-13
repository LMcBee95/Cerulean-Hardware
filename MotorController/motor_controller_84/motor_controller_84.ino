/************************************************************************************************************
 * Motor Controller Sketch
 * IEEE ROV 2014
 * 
 * This sketch is used to control a attiny 84 microcontroller used to control a motor.
 * This sketch also dictates how information is sent and received from the motor controllers.
 *
 * Digital Pins:
 * 0 - tx (sends out serial communication)
 * 1 - readWrite (tells the attiny 84, the slave, whether to be sending or recieving serial communication)
 * 2 - rx (receives serial communication)
 * 3 - Direction (tell the motor which direction to spin)
 * 4 - reset (resets the h-bridge)
 * 5 - speed (how fast the motor goes)
 * 6 - fault2 (Fault check 2)
 * 7 - fault1 (Fault check 1)
 *
 * Under Developement
 ************************************************************************************************************/

#include <SoftwareSerial.h>

//function declarations
byte crc8(const byte *packet);
boolean readPacket(void);
boolean usePacket(void);

//the address of the motor controller
#define ADDRESS 0x01

//different commands of the motor controller
#define CONTROL_MOTOR 0x01
#define STOP 0x02
#define REQUEST_FAULT_DATA 0x03
#define RESET_HBRIDGE 0x04

//Serial Communication pins
#define RX 2
#define TX 0

//Pin numbers for the faults
#define FAULT1 7
#define FAULT2 6

//Pin that tells the attinty 84 to either to recieve data or to write data
#define READWRITE 1

//Pins that tells the motor what speed and direction to operate at
#define POWER 5
#define DIRECTION 3

//Pin that leads to the h-bridge reset pin
#define RESET 4

//variable that stores the fault values
int fault1 = LOW;
int fault2 = LOW;

//packet to store incoming data
// [address, command, argument 1, argument 2, check sum]
byte receivedPacket[] = {0, 0, 0, 0, 0};

//data that is send back to the bottom board
//[start byte, address, command, argument 1, argument 2, check sum, end byte]
byte returnPacket[] = {0x12, 0x00, 0x05, 0, 0, 0, 0x13};

SoftwareSerial mySerial(RX, TX);

void setup()
{
  //Fault Inputs
  pinMode(FAULT1, INPUT);
  pinMode(FAULT2, INPUT);

  //Serail Communication
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT);
  pinMode(READWRITE, INPUT);

  //Information sent to the h-bridge from the attiny 84
  pinMode(DIRECTION, OUTPUT);
  pinMode(RESET, OUTPUT);
  
  //Put the rs485 chip in read mode
  digitalWrite(READWRITE, LOW);
  
  //Put the reset in LOW
  digitalWrite(RESET, LOW);
  
  //begin serial communication
  mySerial.begin(9600);
}

void loop()
{  
  boolean received = readPacket();
  if(received)
  {
   usePacket(); 
  }
}

//function definitions

byte crc8(const byte *packet)
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

boolean readPacket(void)
{
  digitalWrite(READWRITE, LOW);
  
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
                  return false;  //Start byte in wrong location
        }
        else if(nextByte == 0x13 && index != 4)
        {
          return false; //end byte in wrong location
        }

        receivedPacket[index] = mySerial.read();
        index++;
      }

      //Makes sure the last byte it a 13
      byteReceived = mySerial.read();
      if(byteReceived != 0x13)
      {
        return false; //last byte is an end byte
      }

      byte receivedCheck = crc8(receivedPacket);
      if(receivedPacket[4] == receivedCheck)
      {
        while(mySerial.available())
        {
         mySerial.read(); 
        }
        //Function returns true if everything is successful 
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

boolean usePacket(void)
{

  //Checks if this is the correct address
  //If the functions returns true then the device will do something. If it is false then the device will not do anything
 if(receivedPacket[0] == ADDRESS)
 {
   //This is where the specific commands are done.
   
   //Stop the motor
   if(receivedPacket[1] == STOP)
   {
     analogWrite(POWER, 0);
     return true;
   }
   else if(receivedPacket[1] == CONTROL_MOTOR)
   {
     analogWrite(POWER, receivedPacket[2]);
     digitalWrite(DIRECTION, receivedPacket[3]);
     return true;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     returnPacket[3] = digitalRead(FAULT1);
     returnPacket[4] = digitalRead(FAULT2);
     returnPacket[5] = crc8(returnPacket);
     
     //Puts the rs485 chip in write mode, sends data to the bottom board, and puts the rs485 chip in read mode
     digitalWrite(READWRITE, HIGH);
     delay(1);
     for(int i = 0; i < 7; i++)
     {
      mySerial.write(returnPacket[i]); 
     }
     delay(1);
     digitalWrite(READWRITE, LOW);
     return true;
   }
   else if(receivedPacket[1] == RESET_HBRIDGE)
   {
    digitalWrite(RESET, HIGH); 
   }
   else
     return false;
 } 
 else
 {
   return false;
 }
}
