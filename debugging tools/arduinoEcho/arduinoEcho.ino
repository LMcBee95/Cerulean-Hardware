//This sort of extends the serial monitor to work with other non arduino things, such as
//an attiny or what have you.

//Everything written in the Serial monitor will be sent out over pin 3.
//Everything received from pin 2 will the be displayed in the serial monitor.


#include <SoftwareSerial.h>
#define RX 2
#define TX 3

SoftwareSerial tinySerial(RX,TX);
void setup()
{
  pinMode(RX,INPUT);
  pinMode(TX,OUTPUT);
  tinySerial.begin(9600);
  Serial.begin(9600);     //To communicate with user
}

void loop()
{
  char inByte;
  if(Serial.available())
  {
    tinySerial.write(Serial.read());
  }
  if(tinySerial.available())
  {
    Serial.print(tinySerial.read());
  }
}

