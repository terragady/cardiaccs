
//#include <Adafruit_MCP4728.h> //instsalled via library manager
#include <Wire.h>
#include "BMA355.h"
#include "MCP4827.h"

//#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
//  #define Serial SERIAL_PORT_USBVIRTUAL
//#endif

static constexpr bool debug = true; // set false to turn off terminal output

bool initialized = false;
int led_counter = 0;
bool led_value = false;
int print_counter = 0;
float t,dt,fs = 0.0;
int error;
uint16_t ax,ay,az;

int ledPin = 13;
int ldac_pin = 10;
int dac_rdy_pin = 11;

int16_t sensorData[3];
//std::vector<std::array<int16_t, 3>> sensorData;

BMA355 acc;
MCP4827 dac;

void setup() {
  // initialize digital pin 13 as an output.
  pinMode(ledPin, OUTPUT);
  pinMode(ldac_pin, OUTPUT);
  pinMode(dac_rdy_pin, INPUT);

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
}

void loop() {
  
  // run init sequence whenever init flag is false
  while (!initialized) {
    initialized = init_devices();
    delay(1000);
  }

  blink_led();

  // Read CS1
  int res = acc.getAcc(sensorData);
  if (res < 0) {
    if (debug) {
      Serial.print( "Error: BMA355 read fifo failed, code: ");
      Serial.println(res);
    }
    initialized = false;
    return;
  }

  // Set DAC outputs
  if(!dac.fastWrite(sensorData[0], sensorData[1], sensorData[2], 0)) {
    initialized = false; 
    Serial.println("DAC error"); 
    return;
  }

  // DEBUG
  if (debug && (print_counter++ > 1000)) 
  {
    dt = micros() - t;
    t = micros();
    fs = dt/1000.0f;
    
    Serial.print("\tfs: "); Serial.print(1000000.0f/fs); Serial.print(" Hz\t");
    Serial.print("dVal: ");
    for (int i=0;i<3;i++) {
      Serial.print(sensorData[i]);
      Serial.print(" ");
    }
    print_counter = 0;
    Serial.println();
  }

}


//---------------------------------------------------------------

// init sequence function
bool init_devices() {

  if(debug) Serial.println("\nCommence init sequence ...");

  //scanI2C(); // For debugging

  // Set status LED off
  digitalWrite(ledPin, LOW);

  delay(200);

  // initialize the DACs
  if(debug) Serial.print("Initializing DAC... ");

  // Set LDAC pin low
  digitalWrite(ldac_pin, LOW);
  delay(200);

  if(!dac.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X))
    {initialized = false; Serial.println("DAC error"); return false;}
  if(!dac.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X))
    {initialized = false; Serial.println("DAC error"); return false;}
  if(!dac.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X))
    {initialized = false; Serial.println("DAC error"); return false;}
  if(!dac.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X))
    {initialized = false; Serial.println("DAC error"); return false;}

  if(!dac.saveToEEPROM())
    {initialized = false; Serial.println("DAC error"); return false;}

  if(debug) Serial.println("OK");

  digitalWrite(ledPin, HIGH);
  delay(200);

  digitalWrite(ledPin, LOW);
  delay(200);

  if(debug) Serial.print("Initializing CS1... ");
  if (acc.init() != 1) {
    if(debug) Serial.println("Failed");
    digitalWrite(ledPin, HIGH);
    delay(200);
    initialized = false;
    return false;
  }
  else {
    if(debug) Serial.println("OK");
    if(debug) Serial.println("ALL SYSTEMS ARE GO");
    digitalWrite(ledPin, HIGH);
    delay(200);
    return true;
  }
}

void blink_led() {
  // Set status LED, blinking fast means MPU is running
  if (led_counter++ > 100) {
    led_counter = 0;
    if (led_value) {
      digitalWrite(ledPin, LOW);
      led_value = false;
    }
    else {
      digitalWrite(ledPin, HIGH);
      led_value = true;
    }
  }
}
