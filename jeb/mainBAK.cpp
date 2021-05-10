#include <Arduino.h>
#include <Wire.h> //include Wire.h library
#include <M2M_LM75A.h>

M2M_LM75A lm75a;

// define pins
#define EN_5V_PIN 4

// debug
bool debug = true;

int MPU = 0x69;
float AccX, AccY, AccZ, AccW;

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
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x1F); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.requestFrom(MPU, 1, true); // Read 6 registers total, each axis value is stored in 2 registers
  // //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  // AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  // AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // AccW = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  Serial.print(AccX);
  // Serial.print(",");
  // Serial.print(AccY);
  // Serial.print(",");
  // Serial.print(AccZ);
  // Serial.print(",");
  Serial.println("DONE");

  delay(2000); // wait 5 seconds for the next I2C scan
}