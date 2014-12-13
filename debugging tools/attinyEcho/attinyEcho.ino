//Attiny just sends out everything it recieve over serial

#include <SoftwareSerial.h>

//Define pins
#define RX 3
#define TX 4

//Other constants

SoftwareSerial tinySerial(RX,TX);

void setup()
{
  pinMode(RX,INPUT);
  pinMode(TX,OUTPUT);
  tinySerial.begin(9600);
}

void loop()
{
  if(tinySerial.available())
  {
    tinySerial.write(tinySerial.read());
  }
}

