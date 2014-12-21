//This will receive a number of bytes up to PACKET_SIZE
//Once DELAY milliseconds have elapsed or, the PACKET_SIZE number of bytes have
//been received, the attiny will send out the bytes in the same order they were
//received.

#include <SoftwareSerial.h>

//Define pins
#define RX 3
#define TX 4
#define READ_WRITE 0

#define DELAY 20
#define PACKET_SIZE 16
//Other constants

SoftwareSerial tinySerial(RX,TX);

void setup()
{
  pinMode(RX,INPUT);
  pinMode(TX,OUTPUT);
  tinySerial.begin(9600);     //To communicate with user
  pinMode(READ_WRITE,OUTPUT);
  digitalWrite(READ_WRITE,LOW);
}

void loop()
{
  byte packet[PACKET_SIZE];
  byte inByte;
  unsigned long startTime;
  int len = 0;
  int i;
  
  while(!tinySerial.available()){} //wait until there's something to do
  
  startTime = millis();
  while(millis()<startTime+DELAY&&len<PACKET_SIZE)
  {
    if(tinySerial.available())
    {
      packet[len++] = tinySerial.read();
    }
  }
  while(millis()<startTime+DELAY){} //Ensure delay has been met
  digitalWrite(READ_WRITE,HIGH);    //Time to respond
  while(tinySerial.available()){tinySerial.read();} //Empty buffer
  for(i=0;i<len;i++)
  {
    tinySerial.write(packet[i]);
  }
  digitalWrite(READ_WRITE,LOW);    //Time to wait for input again
}

