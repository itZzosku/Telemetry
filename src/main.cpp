/*
  WriteMultipleFields
  
  Description: Writes values to fields 1,2,3,4 and status in a single ThingSpeak update every 20 seconds.
  
  Hardware: ESP32 based boards
  
  !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details. !!!
  
  Note:
  - Requires installation of EPS32 core. See https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md for details. 
  - Select the target hardware from the Tools->Board menu
  - This example is written for a network using WPA encryption. For WEP or WPA, change the WiFi.begin() call accordingly.
  
  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize, and 
  analyze live data streams in the cloud. Visit https://www.thingspeak.com to sign up for a free account and create a channel.  
  
  Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
  See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.
  
  For licensing information, see the accompanying license file.
  
  Copyright 2018, The MathWorks, Inc.
*/

#include <ThingSpeak.h>
#include <WiFi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <credentials.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

char ssid[] = WIFI_SSID; // your network SSID (name)
char pass[] = WIFI_PASS; // your network password
int keyIndex = 0;        // your network key Index number (needed only for WEP)
WiFiClient client;

unsigned long myChannelNumber = Telemetry_CH_ID;
const char *myWriteAPIKey = Telemetry_WRITE_APIKEY;

// Initialize our values
float Temperature = 0;
float Humidity = 0; 
float Pressure = 0;

float TemperatureCalibrated = 0;
float HumidityCalibrated = 0;
float PressureCalibrated = 0;

double mapfloat(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(115200); //Initialize serial

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client); // Initialize ThingSpeak

  while (!Serial);
  Serial.println(F("BME680 sensor found!"));

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop()
{

  // Connect or reconnect to WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      delay(1);
      WiFi.setHostname("Telemetry_ESP32_Node");
      delay(1);
      Serial.println(WiFi.getHostname());
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }


  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Temperature = (bme.temperature);
  TemperatureCalibrated = mapfloat(Temperature, -0.4, 25.64, -1, 25.64);
  Serial.print(Temperature);
  Serial.println(" *C");

  Serial.print("Humidity = ");
  Humidity = (bme.humidity);
  HumidityCalibrated = mapfloat(Humidity, 18.85, 81.73200, 22.118, 85);
  Serial.print(Humidity);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Pressure = (bme.pressure / 100.0);
  PressureCalibrated = (Pressure + 17.40);
  Serial.print(Pressure);
  Serial.println(" hPa");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();


  // set the fields with the values
  ThingSpeak.setField(1, TemperatureCalibrated);
  ThingSpeak.setField(2, HumidityCalibrated);
  ThingSpeak.setField(3, PressureCalibrated);

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200)
  {
    Serial.println("Channel update successful.");
  }
  else
  {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(60000); // Wait 60 seconds to update the channel again
}