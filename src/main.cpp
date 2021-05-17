#include <Arduino.h>
#include <Wire.h> //include Wire.h library
#include <M2M_LM75A.h>
#include "BMA355.h"
#include "ICM42605.h"
#include "I2Cdev.h"

BMA355 cs1;
M2M_LM75A lm75a;
#define I2C_BUS Wire    // Define the I2C bus (Wire instance) you wish to use
I2Cdev i2c_0(&I2C_BUS); // Instantiate the I2Cdev object and point to the desired I2C bus

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
*/
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;
int16_t ICM42605Data[7];   // Stores the 16-bit signed sensor output
ICM42605 ICM42605(&i2c_0); // instantiate ICM42605 class

// define pins
#define EN_5V_PIN 4
#define REDLED 13
#define GREENLED 11
#define BATT_SENSOR 9

// debug
bool debug = false;

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
    ICM42605.reset(); // software reset ICM42605 to default registers
    ICM42605.init(Ascale, Gscale, AODR, GODR);
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
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0)
  {
    if (debug)
    {
      Serial.print("Discovered Sensor Type: ");
      Serial.println("IMC");
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
  if (debug)
    Serial.println("System starting up...");
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
  delay(2000);

  delay(1000);
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
  if (internal_timer > 100000)
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

// here you can control the LED read data freq
  if (internal_timer % 50 == 0 && !red_led_status)
  {
    green_led_status = !green_led_status;
    digitalWrite(GREENLED, green_led_status);
    if (current_voltage < 3.7)
    {
      digitalWrite(REDLED, !green_led_status);
    }
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
  else if (sensor_type == 2)
  {
    ICM42605.readData(ICM42605Data);
    accData[0] = (float)ICM42605Data[1];
    accData[1] = (float)ICM42605Data[2];
    accData[2] = (float)ICM42605Data[3];

    gyroData[0] = (float)ICM42605Data[4];
    gyroData[1] = (float)ICM42605Data[5];
    gyroData[2] = (float)ICM42605Data[6];
  }

  /// writing directly to DAC output
  Wire.beginTransmission(0x0C);
  Wire.write(0b00110000);
  Wire.write(accData[0]);
  Wire.write(accData[0] >> 8);
  Wire.endTransmission();
  Wire.beginTransmission(0x0C);
  Wire.write(0b00110001);
  Wire.write(accData[1]);
  Wire.write(accData[1] >> 8);
  Wire.endTransmission();
  Wire.beginTransmission(0x0C);
  Wire.write(0b00010010);
  Wire.write(accData[2]);
  Wire.write(accData[2] >> 8);
  Wire.endTransmission();
  Wire.beginTransmission(0x0C);
  Wire.write(0b00010011);
  Wire.write(gyroData[0]);
  Wire.write(gyroData[0] >> 8);
  Wire.endTransmission();
  Wire.beginTransmission(0x0C);
  Wire.write(0b00010100);
  Wire.write(gyroData[1]);
  Wire.write(gyroData[1] >> 8);
  Wire.endTransmission();
  Wire.beginTransmission(0x0C);
  Wire.write(0b00010101);
  Wire.write(gyroData[2]);
  Wire.write(gyroData[2] >> 8);
  Wire.endTransmission();
  Wire.endTransmission();
}
