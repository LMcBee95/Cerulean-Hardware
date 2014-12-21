//To use this sketch, connect pin 2 and 3 as the RX and TX pins, and pin 4 as the READ/WRITE pin
//Open up the serial monitor and specify a packet to send in
//The following manner:

// ADDRESS COMMAND VALUE1 VALUE2 <CHECKSUM>!

//All input should terminate with the '!' character.

//Values may be entered in decimal ie 10 or hexadecimal ie 0x0A

// <CHECKSUM> is a flag, if left empty a correct checksum will be
//calculated.  If anything else is used, a corrupt checksum will
//be created and sent.

//Less than 4 arguments will result in random garbage being sent probably
//More than 4 will result in a corrupt checksum


#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

//Define pins
#define RX 2
#define TX 3
#define READ_WRITE 4

//User input related constants
#define MAX_STR_LEN 64
#define MAX_ARGS 7
#define END_CHAR '!'
#define DELIM " "
#define PACKET_LEN 7

//Packet related constants
#define START_BYTE 0x12
#define END_BYTE 0x13

//Other constants
#define TIMEOUT 50 //Number of milliseconds to wait for response before giving up

//Global variables
SoftwareSerial tinySerial(RX,TX);
//String input;
char** input;

//Fuction definitions
char** getUserInput();     //Grab packet from user
void printInput(char**);   //For debugging only, print packet
void destroyInput(char**); //Free allocated memory of packet
void sendPacket(char**);   //Send packet to attiny
byte crc8(const byte *);   //Generate the checksum
void getResponse();        //Get and handle response from attiny

void setup()
{
  pinMode(READ_WRITE,OUTPUT);
  digitalWrite(READ_WRITE,HIGH);
  Serial.begin(9600);     //To communicate with user
  tinySerial.begin(9600); //To communicate with attiny
}

void loop()
{
  input=getUserInput();
  sendPacket(input);
  getResponse();
  destroyInput(input);
}
 
char** getUserInput()
{
  char str[MAX_STR_LEN+1];
  int i=0; //Iterator
  char byteIn;
  boolean terminator = false;
  char** info;
  char* token;

  while(!terminator&&i!=MAX_STR_LEN)
  {
    if(Serial.available()){
      byteIn= Serial.read();
      if(byteIn==END_CHAR){terminator=true;}
      else {str[i]=byteIn;i++;}
    }
  }
  str[i]='\0';
  while(Serial.available()){Serial.read();} //Flush input buffer

  info = (char**)malloc(sizeof(char*)*MAX_ARGS);
  i=0;
  //Time to explodenate WOOOOOOOOOOOOOOOOOOOOO
  token=strtok(str,DELIM);
  while(token!=NULL)
  {
    info[i]=strdup(token);
    token=strtok(NULL,DELIM);
    i++;
  }
  info[i]=strdup("END");
  return info;
}

void printInput(char** info)
{
  int i=0;
  while(strcmp(info[i], "END")!=0)
  {
    Serial.println(info[i]);
    i++;
  } 
}

void destroyInput(char** info)
{
  int i=0;
  while(strcmp(info[i],"END")!=0)
  {
    free(info[i]);
    i++; 
  }
  free(info[i]);
  free(info);
}

void sendPacket(char** info)
{
  byte packet[PACKET_LEN];
  int i;
  boolean corrupt; //Whether or not the checksum should be corrupted
  
  packet[0]=START_BYTE; //Add start byte
  
  for(i=1;i<=PACKET_LEN-3;i++) //Take info from string array and put into packet
  {
    packet[i] = strtol(info[i-1],NULL,0);
  }
  
  //Add checksum
  corrupt = strcmp(info[i-1],"END")==0? false: true;
  packet[i] = crc8(packet);
  if(corrupt){packet[i]= ~packet[i];} //Bitwise not should corrupt it real good
  
  //Add end byte
  packet[++i] = END_BYTE;
  
  //Print info to user
  Serial.print("Sending Packet:");
  for(i=0;i<PACKET_LEN;i++)
  {
    Serial.print(" 0x");
    Serial.print(packet[i],HEX);
  }
  if(corrupt){Serial.print(" (corrupt checksum)");}
  Serial.print('\n');
  
  //Send packet fo-rizzle my nizzle up in hizzle
  for(i=0;i<PACKET_LEN;i++)
  {
    tinySerial.write(packet[i]);
  }
}

//Dont know where this function came from, but hopefully it works
byte crc8(const byte *packet)
{ 
  byte crc = 0;
  *packet++;
  for(byte len = 1; len < PACKET_LEN-2; len++) {
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

//This function assumes that the slave sends back only the packet and no other bytes
//Perhaps this should be made more robust
void getResponse()
{
  byte checksum;
  int len=0; //Number of bytes successfully read
  int i;
  byte packet[PACKET_LEN];
  unsigned long startTime = millis();  //Warning, this will screw up if the arduino has been running for 50 days continuously
  
  digitalWrite(READ_WRITE,LOW);        //Now reading from attiny
  while(millis() <= startTime+TIMEOUT && len<PACKET_LEN)
  {
    if(tinySerial.available())
    {
      packet[len++] = tinySerial.read();
    }
  }
  digitalWrite(READ_WRITE,HIGH);       //No longer reading
  
  while(tinySerial.available()){tinySerial.read();} //Flush remaining buffer
  if(len==0){Serial.print("ERROR: Failed to receive response after ");Serial.print(TIMEOUT,DEC);Serial.print("ms\n");}
  else
  {
    if(len<PACKET_LEN){Serial.print("ERROR: Incomplete packet recieved before timeout!\n");}
    if(len==PACKET_LEN)
    {
      checksum=crc8(packet);
      if(checksum!=packet[PACKET_LEN-2])
      {
        Serial.print("ERROR: Checksum incorrect, should be 0x");
        Serial.print(checksum,HEX); Serial.print(" not 0x");
        Serial.print(packet[PACKET_LEN-2],HEX);Serial.print("\n");
      }
    }
    Serial.print("Packet recieved:");
    for(i=0;i<len;i++)
    {
      Serial.print(" 0x");
      Serial.print(packet[i],HEX);
    }
  }
  Serial.print("\n\n");
  
}
