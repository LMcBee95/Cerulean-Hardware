/*void handleSerialData(char inData[], byte index);
void handleSerialData2(char inData[], byte index);

void welcomeMessage(void);
void ConstructMessage(struct payload *);
byte crc8(const uint8_t msgr);

#define TransmitControl 13   //RS485 Direction control

#define RS485Transmit    HIGH
#define RS485Receive     LOW

String inputString = "";         // a string to hold incoming data
int index= 0; 
boolean stringComplete = false;  // whether the string is complete
boolean terminalConnect = false; // indicates if the terminal has connected to the board yet

char packet[7];

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
  
  pinMode(TransmitControl, OUTPUT);
  digitalWrite(TransmitControl, RS485Receive);
  
  // initialize serial
  Serial.begin(9600);
  Serial.flush();
  Serial1.begin(9600);
  Serial1.flush();

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  if (Serial && !terminalConnect) { 
    welcomeMessage();
    terminalConnect = true;
  } else if (!Serial && terminalConnect) {
    terminalConnect = false;
  }
  
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println(inputString); 
    char buf[inputString.length()+1];
    inputString.toCharArray(buf, inputString.length()+1) ;
  
    // clear the string:
    handleSerialData(buf, index);
    inputString = "";
    index = 0;
    stringComplete = false;

  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    index++; 
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\r') {
      stringComplete = true;
    } 
  }
}


void handleSerialData(char inData[], byte index) {

  // tokenize the input from the terminal by spaces
  char * words[6];
  byte current_word_index = 0;
  char * p = strtok(inData, " ");
  while(p != NULL) {
    words[current_word_index++] = p;
    p = strtok(NULL, " ");
  }
  
  struct payload myData;
  myData.starts = 0x12;
  myData.ends = 0x13;
  myData.chksum = 0x99;
 
  //Address checking
  if(strspn(words[0], "1234567890") <= 1 ){
    Serial.print("Address: ");
    Serial.println(*words[0]);
    
    myData.addr = (byte) atoi(words[0]);    
    //Command Checking
    if(strcmp(words[1], "0") == 0 ) {
      Serial.println("Stop Motors");
      myData.cmd = (byte) atoi(words[1]);
      myData.arg1 = 0xF0;
      myData.arg2 = 0xF0;
      
    }else if(strcmp(words[1], "1") == 0 ) {
      Serial.println("Set Speed");
      myData.cmd = (byte) atoi(words[1]);
      
      //check valid direction
      if(strspn(words[2], "01") <= 1){
        myData.arg1 = (byte) atoi(words[2]);
      }else{
        Serial.println("Invalid Direction");
      }
      
      //print direction
      if(myData.arg1 == 0){
        Serial.println("Motor Forward");
      }else if(myData.arg1 == 1){
        Serial.println("Motor Backward");
      } 
      
      //Check speed values ( from 0 to 127)
      if(strspn(words[3], "1234567890") <=3) {
        byte speedval =(byte) atoi(words[3]);
        
        if(speedval >=128){
          Serial.println("Speed value too high");
        }else if(speedval < 0){
          Serial.println("Speed value too low");
        }else{
          Serial.print("Set speed to:");
          Serial.println(speedval);
          myData.arg2 = speedval; 
        }
        
      }else{
        Serial.println("Invalid Speed Value");
      }
      
      
    }else if(strcmp(words[1], "2") == 0 ){ 
      Serial.println("Request Faults");
      myData.cmd = (byte) atoi(words[1]);
      myData.arg1 = 0x00;
      myData.arg2 = 0xF0;

    }else{
      Serial.println(" Invalid syntax.");
    }
    
    
    
  }else{
    Serial.println("Invalid Address");
  }
 
 ConstructMessage(&myData);
     
}

void ConstructMessage(struct payload * mypayload){
  
  packet[0] = mypayload->starts;
  packet[1] = mypayload->addr;
  packet[2] = mypayload->cmd;
  packet[3] = mypayload->arg1;
  packet[4] = mypayload->arg2;
  packet[5] = mypayload->chksum;
  packet[6] = mypayload->ends;  
  
  //packet[5] = crc8(packet);
  
  Serial.println(packet[6], HEX);
  
  for(byte i=0; i<7; i++){
    Serial.print("\n");
    Serial.print(i, DEC); 
    Serial.print(": "); 
    Serial.println(packet[i], HEX);
  }
  
  Serial.print("\nPrinting CRC:");
//  Serial.println(crc8(packet), HEX);
  
  Serial.println("\nstarting buffer");
  
  digitalWrite(TransmitControl, RS485Transmit);
  
 // for(byte i=0; i<7; i++){
  //  Serial1.write(packet[i]);
  //}  
//  Serial1.flush();
  
  handleSerialData2(packet, 7);
  
  digitalWrite(TransmitControl, RS485Receive);
  
  Serial.println("ending buffer");
  
}


void handleSerialData2(char inData, char index) {
  
  Serial1.print(inData);
  Serial1.write(index);
  
  Serial1.flush();
  
  /*
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

//straight up CRC8 caclulation 
byte crc8(const byte *msg){ 
  
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
}*/

