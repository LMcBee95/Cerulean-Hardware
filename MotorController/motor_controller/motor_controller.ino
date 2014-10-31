/************************************************************************************************************
 * Motor Controller Sketch
 * IEEE ROV 2014
 * 
 * This sketch is used to control a attiny 84 microcontroller used to control a motor.
 * This sketch also dictates how information is sent and received from the motor controllers.
 *
 * Digital Pins:
 * 3 - fault1 (Fault check 1)
 * 4 - fault2 (Fault check 2)
 * 5 - PMW (how fast the motor goes)
 * 6 - reset (resets the h-bridge)
 * 7 - Direction (tell the motor which direction to spin)
 * 8 - rx (receives serial communication)
 * 9 - readWrite (tells the attiny 84, the slave, whether to be sending or recieving serial communication)
 * 10 - tx (sends out serial communication)
 *
 * Under Developement
 ************************************************************************************************************/
 
 #include <SoftwareSerial.h>
 
 //Serial Communication pins
 const int rx = 8;
 const int tx = 10;
 
 SoftwareSerial mySerial(rx,tx);
 
 //Pin numbers for the faults
 const int fault1 = 3;
 const int fault2 = 4;
 
 //Pin that tells the attinty 84 to either to recieve data or to write data
 const int readWrite = 9;
 
 //Pins that tells the motor what speed and direction to operate at
 const int power = 5;
 const int Direction = 7;
 
 //Pin that leads to the h-bridge reset pin
 const int reset = 6;
 
 //Special Identification for each board
 const byte motorNumber = 0x01;
 
 //Start and End byte
 const byte startByte = 0x12;
 const byte endByte = 0x13;
 
 void setup()
 {
   //Fault Inputs
   pinMode(fault1, INPUT);
   pinMode(fault2, INPUT);
   
   //Serail Communication
   pinMode(tx, OUTPUT);
   pinMode(rx, INPUT);
   pinMode(readWrite, INPUT);
   
   //Information sent to the h-bridge from the attiny 84
   pinMode(power, OUTPUT);
   pinMode(Direction, OUTPUT);
   pinMode(reset, OUTPUT);
   
   mySerial.begin(9600);
 }
 
 void loop()
 {  
   if(mySerial.available())
   {
     //check if I can do this
     if(mySerial.read() == startByte)
     {
       if(mySerial.read() == motorNumber)
       {
         
       }
       else
         clearBuffer(endByte);
     }
     else
       clearBuffer(endByte);
   }
 }
 
 void clearBuffer (byte endByte)
 {
  while(mySerial.read() != endByte )
  {
   //Squirels! 
  }
 }
