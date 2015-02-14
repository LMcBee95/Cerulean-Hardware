#include <SoftwareSerial.h>

String str;

SoftwareSerial mySerial(6, 7);

int strstart_P(const char *s1, const char * PROGMEM s2)
{
    return strncmp_P(s1, s2, strlen_P(s2)) == 0;
}

int getdist(void)
{
    char buf[64];
    char *comma;
    int dist;
    int rc;

    for (;;) {
        rc = mySerial.readBytesUntil('\n', buf, sizeof(buf));
        buf[rc] = '\0';

        if (!strstart_P(buf, PSTR("Dist: ")))
            continue;

        comma = strchr(buf, ',');
        if (comma == NULL)
            continue;

        *comma = '\0';

        dist = atoi(buf + strlen_P(PSTR("Dist: ")));
        return dist;
    }
}

int x;

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
   
   if(Serial.available() > 0)
    {
        str = Serial.readStringUntil('\n');
        x = Serial.parseInt();
        Serial.print(x);
    }
    /*dist_mm = getdist();
    dist_m = dist_mm / 1000;

    snprintf_P(buf, sizeof(buf),
    PSTR("Laser distance: %d.%dm"),
    dist_m, dist_mm % 1000);

    Serial.println(buf);*/
   
   delay(5000);
}
