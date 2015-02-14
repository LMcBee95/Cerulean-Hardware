int x;
byte str[200];

#include <SoftwareSerial.h>


SoftwareSerial mySerial(6, 7);

void setup()
{
   Serial.begin(115200); 
   mySerial.begin(115200);

}

void loop()
{
  
    int dist_mm;
    int dist_m;
    char buf[128];  
  
    mySerial.write("*00004#");
   
   while(!mySerial.available());
   
   while(mySerial.available() > 0)
    {
        //str = mySerial.readStringUntil('\n');
        //x = mySerial.parseInt();
        Serial.println(mySerial.read());
    }
    
   delay(5000);
}
