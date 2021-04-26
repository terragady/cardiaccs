#include <Arduino.h>
#include <Wire.h> //include Wire.h library
#include <M2M_LM75A.h>

M2M_LM75A lm75a;

// define pins
#define EN_5V_PIN 4

// debug
bool debug = true;

void setup()
{
  pinMode(EN_5V_PIN, OUTPUT);


  digitalWrite(EN_5V_PIN, HIGH);


  lm75a.begin();
  if (debug)
  {
    Serial.print(F("Starting Temperature: "));
    Serial.print(lm75a.getTemperature());
    Serial.println(F(" *C"));
  }

  Wire.begin(); // Wire communication begin
  Wire.setClock(400000);
  Serial.begin(115200);
}

void loop()
{
  if (debug)
  {
    Serial.print(F("Starting Temperature: "));
    Serial.print(lm75a.getTemperature());
    Serial.println(F(" *C"));
  }

  delay(2000); // wait 5 seconds for the next I2C scan
}