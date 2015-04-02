
int VSENSE1 = 3;
int VSENSE2 = 2;
int VSENSE3 = 1;
int CURR_OUT = 4;

int WATER_CONTROL = 0;
int PWM1 = 7;
int PWM2 = 6;
int LED_PIN = 8;

void setup(){
  pinMode(VSENSE1,INPUT);
  digitalWrite(VSENSE1,HIGH);
  pinMode(VSENSE2,INPUT);
  pinMode(VSENSE3,INPUT);
  pinMode(CURR_OUT,INPUT);
  
  pinMode(WATER_CONTROL,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
}

void update_led(){
  analogWrite(LED_PIN,analogRead(CURR_OUT));
}

void loop(){
//  analogWrite(PWM1,0);
//  for(int i=0;i<255;i++){
//    analogWrite(PWM2,i);
//    update_led();
//    delay(20);
//}
//  delay(2000);
//  analogWrite(PWM2,0);
//
//  for(int i=0;i<255;i++){
//    analogWrite(PWM1,i);
//    update_led();
//    delay(20);
//}
//  delay(1000);
//

digitalWrite(WATER_CONTROL,HIGH);
delay(1000);
digitalWrite(WATER_CONTROL,LOW);
delay(1000);
}
