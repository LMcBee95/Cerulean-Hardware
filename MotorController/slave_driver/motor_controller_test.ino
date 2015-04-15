#include <SoftwareSerial.h>

//function declarations
byte crc8(const byte *packet);
void sendPackets();

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
#define PACKET_SIZE = 7                                                                                                                                                                                                                      

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

//Serial Protocol from bottom board
//[start byte, address, command, argument 1, argument 2, check sum, end byte
byte motor1[] = {0x12, 0x01, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor2[] = {0x12, 0x02, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor3[] = {0x12, 0x03, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor4[] = {0x12, 0x04, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor5[] = {0x12, 0x05, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor6[] = {0x12, 0x06, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor7[] = {0x12, 0x07, CONTROL_MOTOR, 1, 127, 0, 0x013};
byte motor8[] = {0x12, 0x08, CONTROL_MOTOR, 1, 127, 0, 0x013};


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
  digitalWrite(READWRITE, HIGH);
  digitalWrite(RESET, HIGH);
  pinMode(LED, OUTPUT);
  
  
  //Begin serial communication
  mySerial.begin(BAUD_RATE);
 
  digitalWrite(LED, LOW); 

}

void loop()
{ 
  
  sendPackets();
  
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

void sendPackets()
{
  
 motor1[5] = crc8(motor1, 1, 4);
 mySerial.write(motor1[0]);
 mySerial.write(motor1[1]);
 mySerial.write(motor1[2]);
 mySerial.write(motor1[3]);
 mySerial.write(motor1[4]);
 mySerial.write(motor1[5]);
 mySerial.write(motor1[6]);
 
 delay(50);
 
 motor2[5] = crc8(motor2, 1, 4);
 mySerial.write(motor2[0]);
 mySerial.write(motor2[1]);
 mySerial.write(motor2[2]);
 mySerial.write(motor2[3]);
 mySerial.write(motor2[4]);
 mySerial.write(motor2[5]);
 mySerial.write(motor2[6]);
 
 delay(50);
 
 motor3[5] = crc8(motor3, 1, 4);
 mySerial.write(motor3[0]);
 mySerial.write(motor3[1]);
 mySerial.write(motor3[2]);
 mySerial.write(motor3[3]);
 mySerial.write(motor3[4]);
 mySerial.write(motor3[5]);
 mySerial.write(motor3[6]);
 
 delay(50);
 
 motor4[5] = crc8(motor4, 1, 4);
 mySerial.write(motor4[0]);
 mySerial.write(motor4[1]);
 mySerial.write(motor4[2]);
 mySerial.write(motor4[3]);
 mySerial.write(motor4[4]);
 mySerial.write(motor4[5]);
 mySerial.write(motor4[6]);
 
 delay(50);
 
 motor5[5] = crc8(motor5, 1, 4);
 mySerial.write(motor5[0]);
 mySerial.write(motor5[1]);
 mySerial.write(motor5[2]);
 mySerial.write(motor5[3]);
 mySerial.write(motor5[4]);
 mySerial.write(motor5[5]);
 mySerial.write(motor5[6]);
 
 delay(50);
 
 motor6[5] = crc8(motor6, 1, 4);
 mySerial.write(motor6[0]);
 mySerial.write(motor6[1]);
 mySerial.write(motor6[2]);
 mySerial.write(motor6[3]);
 mySerial.write(motor6[4]);
 mySerial.write(motor6[5]);
 mySerial.write(motor6[6]);
 
 delay(50);
 
 motor7[5] = crc8(motor7, 1, 4);
 mySerial.write(motor7[0]);
 mySerial.write(motor7[1]);
 mySerial.write(motor7[2]);
 mySerial.write(motor7[3]);
 mySerial.write(motor7[4]);
 mySerial.write(motor7[5]);
 mySerial.write(motor7[6]);
 
 delay(50);
 
 motor8[5] = crc8(motor8, 1, 4);
 mySerial.write(motor8[0]);
 mySerial.write(motor8[1]);
 mySerial.write(motor8[2]);
 mySerial.write(motor8[3]);
 mySerial.write(motor8[4]);
 mySerial.write(motor8[5]);
 mySerial.write(motor8[6]);
 
 delay(50);
  
  
}
