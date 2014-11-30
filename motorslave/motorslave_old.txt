void handleSerialData(char inData[], byte index);
void welcomeMessage(void);
void ConstructMessage(struct payload *);
byte crc8(const uint8_t msgr);

#define STOP 0
#define SETSPEED 1
#define FAULT 2


#define TransmitControl 12   //RS485 Direction control

#define LED 13

#define RED 4
#define YELLOW 5
#define GREEN 6

#define ADDRESS 0x01

#define RS485Transmit    HIGH
#define RS485Receive     LOW

String inputString = "";         // a string to hold incoming data
int index= 0; 
boolean stringComplete = false;  // whether the string is complete

byte packet[7];

struct payload{ // Payload structure
  byte starts;
  byte addr;
  byte cmd;
  byte arg1;
  byte arg2;
  byte chksum;
  byte ends;
};

void setup() {

  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  pinMode(TransmitControl, OUTPUT);
  digitalWrite(TransmitControl, RS485Receive);

  // initialize serial
  Serial.begin(9600);
  Serial.flush();

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  //Wait for end byte to process data
  if (stringComplete) {
    char buf[inputString.length()+1];
    inputString.toCharArray(buf, inputString.length()+1) ;
    // clear the string:
    handleSerialData(buf, index);
    inputString = "";
    index = 0;
    stringComplete = false;
  }
  
}

//This listens to the serial line for data
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    Serial.print(inChar);
    // add it to the inputString:
    inputString += inChar;
    index++;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == 0x13) {
      stringComplete = true;
    }
  }
}



void handleSerialData(char *inData, byte index) {
  
  Serial.print(inData);
  Serial.write(index);
  
  Serial.flush();
  
  
  if(inData[1] == ADDRESS){
   //digitalWrite(GREEN, HIGH);
    //if(inData[5] == crc8(inData)){
      //digitalWrite(YELLOW, HIGH);
      
   
  
       if(inData[2] == 0x00){      
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
       }else if(inData[2] == 0x01){
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, LOW);
       }else if(inData[2] == 0x02){
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
       }
       
       
    //}else{
      //break or something
    //  digitalWrite(YELLOW, LOW);
    //}
  
  
  }else{
    //ignore everything
    //digitalWrite(GREEN, LOW);
  }
}

void ConstructMessage(struct payload * mypayload){

  packet[0] = mypayload->starts;
  packet[1] = mypayload->addr;
  packet[2] = mypayload->cmd;
  packet[3] = mypayload->arg1;
  packet[4] = mypayload->arg2;
  packet[5] = mypayload->chksum;
  packet[6] = mypayload->ends;  

  Serial.println(packet[6], HEX);

  for(byte i=0; i<7; i++){
    Serial.print("\n");
    Serial.print(i, DEC); 
    Serial.print(": "); 
    Serial.println(packet[i], HEX);
  }


 // Serial.println(crc8(packet), HEX);


  digitalWrite(TransmitControl, RS485Transmit);

  for(byte i=0; i<7; i++){
    Serial.write(packet[i]);
  }
  
  digitalWrite(TransmitControl, RS485Receive);

}

//straight up CRC8 caclulation 
byte crc8(const char *msg){ 

  byte crc = 0;
  for(byte len = 1; len < 5; len++) {
    uint8_t inbyte = *msg++;
    for (byte i = 8; i; i--) {
      byte mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0xD5;
      inbyte >>= 1;
    }
  }
  return crc;
}



void welcomeMessage(void) {
  Serial.print("\r\nWelcome ROV X7 test software\r\n");
  Serial.print("All commands must be terminated with a carriage return.\r\n");
  Serial.print("Type commands in the following format:\r\n");
  Serial.print("Address Command Arg1 Arg2\r\n");
  Serial.print("EX:\"1 1 1 100\"\r\n\n\n");
}


