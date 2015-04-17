/***************************************************************************************************************************************
 * Motor Controller Sketch
 * Purdue IEEE ROV 2014
 * Luke and Ryan McBee
 * 
 * This sketch is used to control a attiny 84 microcontroller used to control a motor.
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
 *  Serial Protocol:
 *   [Start Byte, Address, Command, Argument 1, Argument 2, Check Sum, End Byte]
 *
 * Function Definitions:
 *   crc8 - Creates a check sum using the second, third, fourth, and fifth elements of an array
 *   readPacket -  Reads in a packet sent from the bottom board and puts it in an array if the packet is not corrupted
 *   returnCrc8 -  Creates a check sum using the first, second, thired, and fourth elements of an array
 *   usePacket -  Uses the array created within readPacket in order to tell the microcontroller a specific task to do
 *
 * Commands:
 *   Control Motor - 0x01 - Sets the speed and direction of the motors spin
 *   STOP - 0x02 - Instantly stops the motor
 *   Request - 0x03 - Fault Data - Determines the condition of the motor through two fault sensors and send the data back to the bottom board
 *   Reset H-Bridge - 0x04 - Resets the h-bridge
 *   Send Fault Data - 0x05 - The command given to the packet sent back to the bottom board
 *
 ***************************************************************************************************************************************/

#include <SoftwareSerial.h>

//function declarations
byte crc8(const byte *packet);
byte readPacket(void);
byte usePacket(void);

//Sets what baud rate we are opperating at
#define BAUD_RATE 57600

//Time to allow for read write pin on RS485 to switch states (ms)
//We chose this value based upon testing delay values between 
//1ms and 3ms
#define RS485_DELAY_TIME 2

//Time we chose to set the reset pin high (ms)
#define RESET_DELAY_TIME 10

//the address of the motor controller
#define ADDRESS 0x01                                                                                                                                                                                                                      

//different commands of the motor controller
#define CONTROL_MOTOR 0x01
#define STOP 0x02
#define REQUEST_FAULT_DATA 0x03
#define RESET_HBRIDGE 0x04
#define SEND_FAULT_DATA 0x05

//Pin Numbers
#define TX 0
#define READWRITE 1
#define RX 2
#define DIRECTION 3
#define RESET 4
#define SPEED 5
#define FAULT2 6
#define FAULT1 7
#define LED 8

//variable that stores the fault values
int fault1 = LOW;
int fault2 = LOW;

//packet to store incoming data
// [address, command, argument 1, argument 2, check sum]
byte receivedPacket[] = {0, 0, 0, 0, 0};

//Serial Protocol from bottom board
//[start byte, address, command, argument 1, argument 2, check sum, end byte]
byte returnPacket[] = {0x12, 0x00, SEND_FAULT_DATA, 0, 0, 0, 0x13};

//delay to kill if nothing sent in that time
//40000 is ~1 seconds
#define KILL_DELAY (3 * 40000)
unsigned long long killSwitch = 0;

SoftwareSerial mySerial(RX, TX);

void setup()
{
  //Pin Definitions
  pinMode(FAULT1, INPUT);
  pinMode(FAULT2, INPUT);
  digitalWrite(FAULT1, HIGH);  //internal pullup for fault 1
  digitalWrite(FAULT2, HIGH);  //internal pullup for fault 2
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT);
  pinMode(READWRITE, OUTPUT);
  pinMode(SPEED, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(RESET, OUTPUT);
  digitalWrite(READWRITE, LOW);
  digitalWrite(RESET, HIGH);
  pinMode(LED, OUTPUT);
  
  
  //Begin serial communication
  mySerial.begin(BAUD_RATE);
 
  digitalWrite(LED, LOW); 

}

void loop()
{ 
  byte received = readPacket();
  
  if(received)
  {
   killSwitch = 0;
   usePacket(); 
  }
  
  killSwitch++;
  if(killSwitch > KILL_DELAY)
  {
   killSwitch = 0;
   analogWrite(LED, 0);
   analogWrite(SPEED, 0); 
  }
}

//function definitions
byte crc8(const byte *packet, const byte pos_first_byte, const byte num_bytes_data)
{ 

  byte crc = 0;
  for(byte len = pos_first_byte; len < (pos_first_byte + num_bytes_data); len++) {
    uint8_t inbyte = packet[len];
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
      
      //Makes sure the last byte it a 13
      byteReceived = mySerial.read();
      if(byteReceived != 0x13)
      {
        return 0; //last byte is an end byte
      }
      
      byte receivedCheck = crc8(receivedPacket, 0, 4);
      if(receivedPacket[4] == receivedCheck)
      {
        return 1;  //Yay! Everything Works!
      }
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

byte usePacket(void)
{
  //Checks if this is the correct address
  //If the functions returns true then the device will do something. If it is false then the device will not do anything
 if(receivedPacket[0] == ADDRESS)
 {
   //This is where the specific commands are done.
   
   //Stop the motor
   if(receivedPacket[1] == STOP)
   {
     analogWrite(SPEED, 0);
     return 1;
   }
   else if(receivedPacket[1] == CONTROL_MOTOR)
   {
     analogWrite(LED, receivedPacket[3]);
     analogWrite(SPEED, receivedPacket[3]);
     digitalWrite(DIRECTION, receivedPacket[2]);
     return 1;
   }
   else if(receivedPacket[1] == REQUEST_FAULT_DATA)
   {
     returnPacket[3] = !(digitalRead(FAULT1));
     returnPacket[4] = !(digitalRead(FAULT2));
     returnPacket[5] = crc8(returnPacket, 1, 4);
     
     
     //Puts the rs485 chip in write mode, sends data to the bottom board, and puts the rs485 chip in read mode
     digitalWrite(READWRITE, HIGH);
     delay(RS485_DELAY_TIME);
     for(int i = 0; i < 7; i++)
     {
       mySerial.write(returnPacket[i]);
     }
     delay(RS485_DELAY_TIME);
     digitalWrite(READWRITE, LOW);
        
     return 1;
   }
   else if(receivedPacket[1] == RESET_HBRIDGE)
   {
    digitalWrite(RESET, HIGH); 
    delay(RESET_DELAY_TIME);
    digitalWrite(RESET, LOW);
    return 1;
   }
   else 
     return 0; //Undefined Motor Command
 } 
 else
   return 0; //Motor Address Was Incorrectu
}
