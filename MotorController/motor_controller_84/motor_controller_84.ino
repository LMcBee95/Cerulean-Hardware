/************************************************************************************************************
 * Motor Controller Sketch
 * Purdue IEEE ROV 2014
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

//Figure out what values signify a fault

#include <SoftwareSerial.h>

//function declarations
byte crc8(const byte *packet);
boolean readPacket(void);
boolean usePacket(void);

//Sets what baud rate we are opperating at
#define BAUD_RATE 28800


//the address of the motor controller
#define ADDRESS 0x04

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
  pinMode(READWRITE, OUTPUT);

  //Information sent to the h-bridge from the attiny 84
  pinMode(DIRECTION, OUTPUT);
  pinMode(RESET, OUTPUT);
  
  //Put the rs485 chip in read mode
  digitalWrite(READWRITE, LOW);
  
  //Put the reset in LOW
  digitalWrite(RESET, LOW);
  
  //begin serial communication
  mySerial.begin(BAUD_RATE);
  
  pinMode(8, OUTPUT);
 
  digitalWrite(8, LOW); 

}

void loop()
{ 
  byte received = readPacket();
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

byte readPacket(void)
{
  digitalWrite(READWRITE, LOW);
  
  //storage variables for the function
  byte byteReceived = 0;
  int index = 0;
  byte nextByte = 0; 
    
  if(mySerial.available() >= 7)
  {
    //Dealy to compensate for the speed difference between the serial and how fast the attiny is going.
    //Checks if the first byte is a start byte.
    if(mySerial.read() == 0x12)
    {      
      index = 0;
      //Reads the five important bytes into the array
      while(index < 5)
      {
        //Makes sure there are no random 12 or 13 in the packet
        nextByte = mySerial.peek();
        if(nextByte == 0x12)
        { 
          return 0;  //Start byte in wrong location
        }
        else if(nextByte == 0x13 && index != 4)
        {
          return 0; //end byte in wrong location
        }

        receivedPacket[index] = mySerial.read();
        index++;
      }
      
      //mySerial.write(receivedPacket[0]);
      
      //Makes sure the last byte it a 13
      byteReceived = mySerial.read();
      if(byteReceived != 0x13)
      {
        return 0; //last byte is an end byte
      }
      
      byte receivedCheck = crc8(receivedPacket);
      if(receivedPacket[4] == receivedCheck)
      {
        return 1;  //Yay! Everything Works!
      }
      digitalWrite(8, HIGH);
      return 0;  //check sum found an error
    }   

    //Retruns a number if the first byte is not a 12
    else
    {
      return 0; //first byte of buffer is not start byte
    }
  }
  else
  {
    return 0;
  }
} 

byte returnCrc8(const byte *packet)
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
     digitalWrite(8, LOW);
     analogWrite(POWER, receivedPacket[2]);
     digitalWrite(DIRECTION, receivedPacket[3]);
     return true;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     returnPacket[3] = digitalRead(FAULT1);
     returnPacket[4] = digitalRead(FAULT2);
     returnPacket[5] = returnCrc8(returnPacket);
     
     
     //Puts the rs485 chip in write mode, sends data to the bottom board, and puts the rs485 chip in read mode
     digitalWrite(READWRITE, HIGH);
     delay(2);
     for(int i = 0; i < 7; i++)
     {
       mySerial.write(returnPacket[i]);
     }
     delay(2);
     digitalWrite(READWRITE, LOW);
        
     return true;
   }
   else if(receivedPacket[1] == RESET_HBRIDGE)
   {
    digitalWrite(RESET, HIGH); 
    delay(10);/************************************************************************************************************
 * Motor Controller Sketch
 * Purdue IEEE ROV 2014
 * 
 * This sketch uses an attiny 84 microcontroller to control a motor.
 * This sketch also dictates how information is sent and received from the motor controllers.
 *
 * Digital Pins:
 *   0 - tx (sends out serial communication)
 *   1 - readWrite (tells the attiny 84, the slave, whether to be sending or recieving serial communication)
 *   2 - rx (receives serial communication)
 *   3 - Direction (tell the motor which direction to spin)
 *   4 - reset (resets the h-bridge)
 *   5 - speed (how fast the motor goes)
 *   6 - fault2 (Fault check 2)
 *   7 - fault1 (Fault check 1)
 *
 * Protocol for Serial Packets:
 *   [Start Byte, Address, Command, Argument 1, Argument 2, Check Sum, End Byte]
 *
 * Function Definitions:
 *   crc8 - Creates a check sum using the second, third, fourth, and fifth elements of an array
 *   readPacket -  Reads in a packet sent from the bottom board and puts it in an array if the packet is not corrupted
 *   returnCrc8 -  Creates a check sum using the first, second, thired, and fourth elements of an array
 *   usePacket -  Uses the array created within readPacket in order to tell the microcontroller a specific task to do
 *
 * Commands:
 *   Control Motor - Sets the speed and direction of the motors spin
 *   STOP - Instantly stops the motor
 *   Request Fault Data - Determines the condition of the motor through two fault sensors and send the data back to the bottom board
 *   Reset H-Bridge - Resets the h-bridge
 *   Send Back Fault Data - The command given to the packet sent back to the bottom board
 *
 ************************************************************************************************************/

#include <SoftwareSerial.h>

//function declarations
byte crc8(const byte *packet);
boolean readPacket(void);
boolean usePacket(void);

//Sets Baud Rate
#define BAUD_RATE 28800

//Address of Motor Controller
#define ADDRESS 0x01

//Different Commands
#define CONTROL_MOTOR 0x01
#define STOP 0x02
#define REQUEST_FAULT_DATA 0x03
#define RESET_HBRIDGE 0x04
#define SEND_BACK_FAULT_DATA 0x05

//Serial Communication Pins
#define RX 2
#define TX 0

//Fault Pin Numbers
#define FAULT1 7
#define FAULT2 6

//Pin that tells the attinty 84 to either to recieve data or to write data
#define READWRITE 1

//Pins That Control the Motor
#define SPEED 5
#define DIRECTION 3

//Pin That Leads to H-Bridge Reset
#define RESET 4

//Led Pin
#define LED 8

//Storage of Fault Data
int fault1 = LOW;
int fault2 = LOW;

//Storage of Recieved Data
byte receivedPacket[] = {0, 0, 0, 0, 0};

//Storage of Data to Be Sent Back
byte returnPacket[] = {0x12, 0x00, SEND_BACK_FAULT_DATA, 0, 0, 0, 0x13};

SoftwareSerial mySerial(RX, TX);

void setup()
{
  //Begin Serial Communication
  mySerial.begin(BAUD_RATE);
  
  //Pin Definitions
  pinMode(FAULT1, INPUT);
  pinMode(FAULT2, INPUT);
  pinMode(READWRITE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(SPEED, OUTPUT);
  
  //Put the rs485 Chip in Read Mode
  digitalWrite(READWRITE, LOW);
  
  // Initially Set Reset and Led Pins Low
  digitalWrite(RESET, LOW); 
  digitalWrite(8, LOW); 

}

void loop()
{ 
  byte received = readPacket();  //See if we recieved a packet
  if(received)  
  {
   usePacket(); //Use the information from the packet 
  }
}

//Function Definitions

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

byte readPacket(void)
{
  digitalWrite(READWRITE, LOW);
  
  //storage variables for the function
  byte byteReceived = 0;
  int index = 0;
  byte nextByte = 0; 
    
  if(mySerial.available() >= 7)
  {
    if(mySerial.read() == 0x12) //Checks if the first byte is a start byte.
    {      
      index = 0;
      while(index < 5) //Reads the five important bytes into the array
      {
        nextByte = mySerial.peek();
        if(nextByte == 0x12) //Makes sure there are no random 12 or 13 in the packet
        { 
          return 0;  //Start byte in wrong location
        }
        else if(nextByte == 0x13 && index != 4)
        {
          return 0; //end byte in wrong location
        }

        receivedPacket[index] = mySerial.read();
        index++;
      }

      byteReceived = mySerial.read();
      if(byteReceived != 0x13) //Makes sure the last byte it a 13
      {
        return 0; //last byte is an end byte
      }
      
      byte receivedCheck = crc8(receivedPacket);
      if(receivedPacket[4] == receivedCheck) //Function returns true if everything is successful 
      {
        while(mySerial.available()) //Clear the Serial Buffer
        {
           mySerial.read(); 
        }
        return 1;  //Yay! Everything Works!
      }
      return 0;  //Check Sum Found an Error
    }   
    else
    {
      return 0; //first byte of buffer is not start byte
    }
  }
  else
    return 0;
} 

byte returnCrc8(const byte *packet)
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

boolean usePacket(void)
{
 if(receivedPacket[0] == ADDRESS)   //Checks if this is the correct address
 {
   

   if(receivedPacket[1] == CONTROL_MOTOR) // Control the Speed and Direction of the Motor's Spin
   {
     digitalWrite(DIRECTION, receivedPacket[2]);
     analogWrite(SPEED, receivedPacket[3]);
     return true;
   }
   else if(receivedPacket[1] == STOP) //Stop the Motor
   {
     analogWrite(SPEED, 0);
     return true;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     returnPacket[3] = digitalRead(FAULT1);
     returnPacket[4] = digitalRead(FAULT2);
     returnPacket[5] = returnCrc8(returnPacket);

     digitalWrite(READWRITE, HIGH); //Put the rs485 chip in write mode
     delay(1); //Remove noise from switching readWrite pin too quickly
     for(int i = 0; i < 7; i++) //Sends Data to the Bottom Board
     {
       mySerial.write(returnPacket[i]);
     }
     
     delay(1); //Remove noise from switching readWrtie pin too quickly
     digitalWrite(READWRITE, LOW); //Put the rs485 chip in read mode
     
     return true;
   }
   else if(receivedPacket[1] == RESET_HBRIDGE)
   {
    digitalWrite(RESET, HIGH); 
    delay(10);
    digitalWrite(RESET, LOW);
    return true;
   }
   else 
     return false; //Undefined Motor Command
 } 
 else
   return false; //Motor Address Was Incorrect
}
    digitalWrite(RESET, LOW);
   }
   else
     return false;
 } 
 else
 {
   return false;
 }
}
