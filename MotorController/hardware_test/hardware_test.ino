int FAULT1_PIN = 7;
int FAULT2_PIN = 6;
int LED_PIN = 8;
int PWM_PIN = 5;
int DIR_PIN = 3;
int RST_PIN = 4;


void setup(){
  pinMode(FAULT1_PIN,INPUT);
  //set internal pullup high
  digitalWrite(FAULT1_PIN,HIGH);
  pinMode(FAULT2_PIN,INPUT);
  //set internal pullup high
  digitalWrite(FAULT2_PIN,HIGH);
  pinMode(LED_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(RST_PIN,OUTPUT);
  digitalWrite(RST_PIN,HIGH);
}

void loop(){
  if(!analogRead(FAULT1_PIN) || !analogRead(FAULT2_PIN)){
    digitalWrite(LED_PIN,HIGH);
    delay(100);
    digitalWrite(LED_PIN,LOW);
  }
  digitalWrite(DIR_PIN,0);
  analogWrite(PWM_PIN,0);
  delay(5000);
  analogWrite(PWM_PIN,128);
  delay(5000);
  digitalWrite(RST_PIN,LOW);
  delay(10);
  digitalWrite(RST_PIN,HIGH);
}

