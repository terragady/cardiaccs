#include <Arduino.h>
#include <M2M_LM75A.h>
M2M_LM75A lm75a;


void setup() {

  pinMode(13, OUTPUT);
  lm75a.begin();
      digitalWrite(13, HIGH);

  
  delay(1000);
  Serial.begin(9600);
  delay(1000);

}

void loop() {


  delay(1000);
    digitalWrite(13, LOW);
    
  delay(1000);
    digitalWrite(13, HIGH);


  delay(3000);
pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

}