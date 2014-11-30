#define STOP 0
#define SETSPEED 1
#define FAULT 2

#define RED 4
#define YELLOW 5
#define GREEN 6

#define TransmitControl 12   //RS485 Direction control

#define ADDRESS 0x01

#define RS485Transmit    HIGH
#define RS485Receive     LOW

char command[8];
boolean commandComplete = false;
boolean lights[3];
byte index = 0;

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  pinMode(TransmitControl, OUTPUT);
  digitalWrite(TransmitControl, RS485Receive);

  // initialize serial
  Serial.begin(9600);
  Serial.flush();
}

void loop() {
  if(Serial.available()) {
    char c = (char)Serial.read();
    Serial.println(c);
    
    //digitalWrite(RED, HIGH);
    
    
    if(c == '0') {
      if(lights[0]) {
        digitalWrite(RED, LOW);      
      } else {
        digitalWrite(RED, HIGH);
      }
      lights[0] = !lights[0];
    }
    else if(c == '1') {
      if(lights[1]) {
        digitalWrite(YELLOW, LOW);      
      } else {
        digitalWrite(YELLOW, HIGH);
      }
      lights[1] = !lights[1];
    } 
    else if(c == '2') {
      if(lights[2]) {
        digitalWrite(GREEN, LOW);      
      } else {
        digitalWrite(GREEN, HIGH);
      }
      lights[2] = !lights[2];
    }
  }
}

void toggleLight(byte i) {
  if(lights[i]) {
    digitalWrite(i+4, HIGH);
  } else {
    digitalWrite(i+4, LOW);
  }
  lights[i] = !lights[i];
}

/*
// This is called by Arduino whenever there is data
void serialEvent() {
  while (Serial.available()) {
    command[index] = (char)Serial.read();
    Serial.print(inChar);
    index++;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == 0x13) {
      commandComplete = true;
      index = 0;
    }
  }
}
*/
