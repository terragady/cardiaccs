#include <Arduino.h>
#include <Wire.h> //include Wire.h library
#include <M2M_LM75A.h>
#include "BMA355.h"

BMA355 cs1;
M2M_LM75A lm75a;

// define pins
#define EN_5V_PIN 4
#define REDLED 13
#define GREENLED 11
#define BATT_SENSOR 9

// debug
bool debug = true;

// variables
bool initialized;
int sensor_type = 0;
int16_t accData[3];
int16_t gyroData[3];
signed int internal_timer = 0;
bool green_led_status = false;
bool red_led_status = false;
float current_voltage = 5.00;

bool init_sensor()
{
  if (debug)
    Serial.println("\nCommence init sequence ...");
  if (debug)
    Serial.println("OK");
  if (debug)
    Serial.print("Initializing sensor......");
  if (sensor_type == 1)
  {
    if (cs1.init() != 1)
    {
      if (debug)
        Serial.println("Failed");
      delay(200);
      initialized = false;
      return false;
    }
    else
    {
      if (debug)
        Serial.println("OK");
      if (debug)
        Serial.println("CS1 was initialized successfully!");
      delay(200);
      return true;
    }
  }
  else if (sensor_type == 2)
  {
    return true;
    // CS2 sensor init procedure
  }
  else
  {
    return false;
  }
}
int disc_sensor()
{

  // checking if CS1 responds with success
  Wire.beginTransmission(0x19);
  if (Wire.endTransmission() == 0)
  {
    if (debug)
    {
      Serial.print("Discovered Sensor Type: ");
      Serial.println("BMA355");
    }
    return 1;
  }
  //check if CS2 responds with success
  Wire.beginTransmission(0x00);
  if (Wire.endTransmission() == 0)
  {
    if (debug)
    {
      Serial.print("Discovered Sensor Type: ");
      Serial.println("MPU9250");
    }
    return 2;
  }

  return 0;
}
float check_voltage()
{
  float volt = analogRead(BATT_SENSOR);
  volt *= 2;    // we divided by 2, so multiply back
  volt *= 3;    // Multiply by 3.3V, our reference voltage
  volt /= 4096; // convert to voltage
  if (debug)
    Serial.print("System Voltage is: ");
  if (debug)
    Serial.println(volt);
  return volt;
}
void setup()
{
  pinMode(EN_5V_PIN, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  analogReadResolution(12);

  Wire.begin(); // Wire communication begin
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(2000);
  current_voltage = check_voltage();
  while (current_voltage < 3.4)
  {
    if (debug)
      Serial.println("Voltage is lower than 3.2");
    digitalWrite(REDLED, HIGH);
    delay(100);
    digitalWrite(REDLED, LOW);
    delay(3000);
    digitalWrite(REDLED, HIGH);
    delay(100);
    digitalWrite(REDLED, LOW);
    delay(3000);
    digitalWrite(REDLED, HIGH);
    delay(100);
    digitalWrite(REDLED, LOW);
    delay(3000);
    current_voltage = check_voltage();
  }
  digitalWrite(REDLED, LOW);
  digitalWrite(EN_5V_PIN, HIGH);


  lm75a.begin();
  if (debug)
  {
    Serial.print(F("Starting Temperature: "));
    Serial.print(lm75a.getTemperature());
    Serial.println(F(" *C"));
  }

  if (debug)
    delay(3000);

  // sensor type :  1 - BMA355 ; 2 - something other
  sensor_type = disc_sensor();
  while (sensor_type == 0)
  {
    digitalWrite(REDLED, LOW);
    delay(150);
    if (debug)
      Serial.println("Sensor not discovered, trying again!");
    digitalWrite(REDLED, HIGH);
    delay(150);

    sensor_type = disc_sensor();
  }
  digitalWrite(REDLED, LOW);

  while (!init_sensor())
  {
    delay(1000);
    red_led_status = !red_led_status;
    digitalWrite(REDLED, red_led_status);
    if (debug)
      Serial.println("Failed init, trying to init again!");
  }
  red_led_status = false;
  digitalWrite(REDLED, LOW);

  check_voltage();

  if (debug)
    Serial.println("Setup Completed. Device is starting reading data.");
  digitalWrite(GREENLED, HIGH);
  delay(150);
  digitalWrite(GREENLED, LOW);
  delay(100);
  digitalWrite(GREENLED, HIGH);
  delay(150);
  digitalWrite(GREENLED, LOW);
  delay(100);
  digitalWrite(GREENLED, HIGH);
  delay(150);
  digitalWrite(GREENLED, LOW);
  delay(1000);
}

void loop()
{
  ////// internal timer/////
  internal_timer++;
  if (internal_timer > 1000)
    internal_timer = 1;
  //////////////////////////

  if (internal_timer == 1)
  {
    current_voltage = check_voltage();
    while (current_voltage < 3.4)
    {
      if (debug)
        Serial.println("Voltage is lower than 3.2");
      digitalWrite(EN_5V_PIN, LOW);

      digitalWrite(REDLED, HIGH);
      delay(100);
      digitalWrite(REDLED, LOW);
      delay(3000);
      digitalWrite(REDLED, HIGH);
      delay(100);
      digitalWrite(REDLED, LOW);
      delay(3000);
      digitalWrite(REDLED, HIGH);
      delay(100);
      digitalWrite(REDLED, LOW);
      delay(3000);
      current_voltage = check_voltage();
    }
    digitalWrite(EN_5V_PIN, HIGH);
  }

  if (internal_timer % 200 == 0 && !red_led_status)
  {
    green_led_status = !green_led_status;
    digitalWrite(GREENLED, green_led_status);
    if (current_voltage < 3.7)
      digitalWrite(REDLED, !green_led_status);
  }

  if (sensor_type == 1)
  {
    int res = cs1.getAcc(accData);
    if (res < 0)
    {
      if (debug)
      {
        Serial.print("Error: BMA355 read fifo failed, code: ");
        Serial.println(res);
      }
      digitalWrite(REDLED, HIGH);

      while (!init_sensor())
      {
        delay(1000);
      }
      digitalWrite(REDLED, LOW);
      return;
    }
  }
}
