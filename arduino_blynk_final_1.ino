// BLYNK CREDENTIALS
#define BLYNK_TEMPLATE_ID     "TMPL3x-af2Kq3"
#define BLYNK_TEMPLATE_NAME   "TESTING"
#define BLYNK_AUTH_TOKEN      "dQ5ENO6Dp5TNnGp6B_47BOYh6YL2pNlb"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// TEMPERATURE SENSOR 
// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into port 7 on the Arduino
#define ONE_WIRE_BUS 7
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
double tempC;

//TDS SENSOR
#include <EEPROM.h>
#include "GravityTDS.h"
#define TdsSensorPin A0
GravityTDS gravityTds;
double finalTds;

// PH SENSOR
float calib=22.80;// value is set according to ouput
int analogPh; // to read analog input value of p
int reading=A1;
double finalPh;

//RTC MODULE
#include <RTClib.h>
unsigned long foggerStartTime; // Stores the start time for fogger activation
unsigned long foggerEndTime; // Stores the end time for fogger activation (after 45 minutes)
unsigned long cooldownEndTime; // Stores the end time for cooldown period (after 15 minutes)
RTC_DS3231 rtc;

//FOGGERS
int foggerPin;



// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Bellams wifi";
char pass[] = "Vamshi123";

// Hardware Serial on Mega, Leonardo, Micro...
//#define EspSerial Serial1

// or Software Serial on Uno, Nano...
#include <SoftwareSerial.h>
SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);


void setup()
{
  Serial.begin(115200);
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);

  // TEMPERATURE SENSOR
  // Start up the library
  sensors.begin();


  //TDS SENSOR
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  // PH SENSOR
  pinMode(reading,OUTPUT);
  
}


//FUNCTION TO READ TEMPERATURE VALUE
void tempRead()
{
   // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  tempC = sensors.getTempCByIndex(0);
  Blynk.virtualWrite(V0, tempC);

  {
    Serial.print("Temperature is: ");
    Serial.println(tempC);
    delay(4000);
  }
}

//FUNCTION TO READ TDS VALUE
void tdsRead()
{
  //TDS SENSOR
    //temperature = readTemperature();  //add your temperature sensor and read it
    //gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    finalTds = gravityTds.getTdsValue();  // then get the value
    Serial.print("TDS VALUE= ");
    Serial.print(finalTds);
    Serial.println("ppm");
    delay(3000);
    Blynk.virtualWrite(V1, finalTds);
}

//FUNCTION TO READ PH VALUE
void phRead()
{
analogPh = analogRead(reading);
float phVoltage=analogPh*5.0/1024;
finalPh=-5.70 * phVoltage +calib;
Serial.print("PH value");
Serial.print(finalPh);
Serial.print("ph");
Blynk.virtualWrite(V2, finalPh);
}


int motorSpeed =0; // Set desired speed of motor

// SETTING MODES
struct Mode {
  int motor_Pin;
  int motor_Speed;
  double required_Ph;
  double required_Tds;
  
};

Mode modes[4] = {
  {6.0, 2.0, 7.0, 300}, //  Mode 1 (motor pin,motor speed,requiredPh, requiredtds)
  {9.0, 2.0, 6.5, 500}, //  Mode 2 
  {10.0,2.0, 7.0, 300}, //  Mode 3
  {11.0,2.0, 6.5, 500}, //  Mode 4 
};

int currentMode = 0; // Stores the currently active mode index

BLYNK_WRITE(V3) {
  currentMode = 0; // Set current  Mode 1 -default mode
  adjustValuesBasedOnMode();
}
BLYNK_WRITE(V4) {
  currentMode = 1; // Set current Mode 2
  adjustValuesBasedOnMode();
}
BLYNK_WRITE(V5) {
  currentMode = 2; // Set current Mode 3
  adjustValuesBasedOnMode(); 
}
BLYNK_WRITE(V6) {
  currentMode = 3; // Set current Mode 4
  adjustValuesBasedOnMode();
}

// Function to run motors and adjust values based on mode selected

bool adjustValuesBasedOnMode() {
 
  if (abs(finalPh - modes[currentMode].required_Ph) > 0.2 || abs(finalTds - modes[currentMode].required_Tds) > 50) {
    // Start or adjust motor speed based on deviation from target values
    analogWrite(modes[currentMode].motor_Pin, modes[currentMode].motor_Speed); // Adjust motor speed if needed
    return false; // Motor needs to run or adjust speed (not at target)
  } else {
    // Stop motor if both pH and TDS are within acceptable range
    analogWrite(modes[currentMode].motor_Pin, 0);
    return true; // Motor stopped (reached target)
  }
}

// TO TURN ON FOGGERS FOR 45 AND TURN OF FOR 15 MINUTES AND OR BASED ON TEMPERATURE VALUE


void activateMode() {
  foggerStartTime = rtc.now().unixtime(); // Store start time for fogger activation
  foggerEndTime = foggerStartTime + (45 * 60); // Calculate end time after 45 minutes
  cooldownEndTime = foggerEndTime + (15 * 60); // Calculate end time for cooldown (after 15 minutes)
  digitalWrite(foggerPin, HIGH); // Start foggers (adjust logic if needed)

  } 
void foggerOff()
{
  if (tempC > 30 || rtc.now().unixtime() >= foggerEndTime) {
    // Stop foggers and cooldown if temperature exceeds 30 degrees OR Stop foggers after 45 minutes and enter cooldown
    digitalWrite(foggerPin, LOW); // Stop foggers (adjust logic if needed)
  } else if (rtc.now().unixtime() >= cooldownEndTime) {
    // Reset fogger state after cooldown period
    foggerStartTime = 0;
    foggerEndTime = 0;
    cooldownEndTime = 0;

  }
}

void loop()
{
  
  Blynk.run();
  tempRead();
  tdsRead();
  phRead();
  activateMode();
  foggerOff();


}