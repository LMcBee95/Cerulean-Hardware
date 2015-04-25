int FAULT1_PIN = 7;
int FAULT2_PIN = 6;
int LED_PIN = 8;
int PWM_PIN = 5;
int DIR_PIN = 3;
int RST_PIN = 4;


void setup(){
pinMode(RST_PIN,OUTPUT);
digitalWrite(RST_PIN,LOW);
pinMode(DIR_PIN,OUTPUT);
pinMode(PWM_PIN,OUTPUT);
analogWrite(PWM_PIN,0);
digitalWrite(DIR_PIN,LOW);
pinMode(LED_PIN,OUTPUT);

pinMode(FAULT1_PIN,INPUT);
digitalWrite(FAULT1_PIN,HIGH);
pinMode(FAULT2_PIN,INPUT);
digitalWrite(FAULT2_PIN,HIGH);

digitalWrite(RST_PIN,HIGH);
}

void check_fault(){
    if(digitalRead(FAULT1_PIN)){
    for(int i=0;i<5;i++){
    digitalWrite(LED_PIN,HIGH);
    delay(100);
    digitalWrite(LED_PIN,LOW);
    delay(100);
    }
  }
  if(digitalRead(FAULT2_PIN)){
    for(int i=0;i<2;i++){
    digitalWrite(LED_PIN,HIGH);
    delay(500);
    digitalWrite(LED_PIN,LOW);
    delay(500);
    }
  }
}
  
void reset(){
  digitalWrite(RST_PIN,LOW);
  delay(10);
  digitalWrite(RST_PIN,HIGH);
}

void loop(){
  digitalWrite(DIR_PIN,HIGH);
  analogWrite(PWM_PIN,100);
  delay(1000);
  analogWrite(PWM_PIN,0);
  delay(1000);
  check_fault();
  reset();
  digitalWrite(DIR_PIN,LOW);
  analogWrite(PWM_PIN,100);
  delay(1000);
  analogWrite(PWM_PIN,0);
  delay(1000);
  check_fault();
  reset();
}
