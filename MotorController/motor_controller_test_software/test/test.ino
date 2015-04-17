 /*
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
 */
 
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

SoftwareSerial mySerial(RX, TX);

void setup()
{
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT);
  pinMode(READWRITE, OUTPUT);
  pinMode(RESET, OUTPUT);
  digitalWrite(READWRITE, HIGH);
  digitalWrite(RESET, HIGH);
  pinMode(LED, OUTPUT);
  
  
  //Begin serial communication
  mySerial.begin(BAUD_RATE);
  Serial.begin(BAUD_RATE);
  digitalWrite(LED, HIGH); 

}

void loop() { 
  sendPacket();
}

//function definitions
byte crc8(const byte *packet, const byte pos_first_byte, const byte num_bytes_data) { 

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
  
byte sendPacket(void) {

     byte sendPacket[] = {0x12, 0, 0x01, 1, 25, 0, 0x13};
     for (int i = 1; i <= 8; i ++) {
       sendPacket[1] = i;
       sendPacket[5] = crc8(sendPacket, 1, 5);
       delay(RS485_DELAY_TIME);
       for(int i = 0; i < 7; i++) {
         Serial.write(returnPacket[i]);
       }
    
       return 1;
}
