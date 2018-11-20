
#include <Arduino.h>
#include "UltraSonic.h"

constexpr int us_pin = 6;


void setup()
{
  Serial.begin(115200);
  while(!Serial);
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  UltraSonic us(us_pin, 500);

  us.init();


  while(1){
      Serial.println(us.total_time_us);
      delay(20);
  }
  
}

void loop()
{
  
}
