/*
TODO
Stacked branch

*/

#include <WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <ArduinoJson.h>
#include "Adafruit_seesaw.h"

#include <credentials.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

#define I2C_SDA 23
#define I2C_SCL 22

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_seesaw ss;

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Update these with values suitable for your network.
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const char *mqtt_server = "192.168.1.5";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
#define MSG_BUFFER_SIZE	(200)
char msg[MSG_BUFFER_SIZE];
int value = 0;

// Initialize our values
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;

float TemperatureCalibrated = 0;
float HumidityCalibrated = 0;
float PressureCalibrated = 0;

float tempCCalibrated = 0;
float capreadCalibrated = 0;

String JSONmessage;

double mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  delay(1);
  WiFi.setHostname("Plant_Telemetry_ESP32_Node");
  delay(1);
  Serial.println(WiFi.getHostname());

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  }
  else
  {
    digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off by making the voltage HIGH
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  StaticJsonDocument<200> doc;
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!Serial)
    ;
  Serial.println(F("BME680 sensor found!"));

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println("seesaw Soil Sensor example!");
  
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
}

void loop()
{

  long now = millis();
  if (now - lastMsg > 30000)
  {
    lastMsg = now;

    if (!client.connected())
    {
      reconnect();
    }
    client.loop();

    if (!bme.performReading())
    {
      Serial.println("Failed to perform reading :(");
      return;
    }

    Serial.print("Temperature = ");
    Temperature = (bme.temperature);
    TemperatureCalibrated = mapfloat(Temperature, -0.4, 25.64, -1, 25.64);
    Serial.print(TemperatureCalibrated);
    Serial.println(" *C");

    Serial.print("Humidity = ");
    Humidity = (bme.humidity);
    HumidityCalibrated = mapfloat(Humidity, 18.85, 81.73200, 22.118, 85);
    Serial.print(HumidityCalibrated);
    Serial.println(" %");

    Serial.print("Pressure = ");
    Pressure = (bme.pressure / 100.0);
    PressureCalibrated = (Pressure + 17.40);
    Serial.print(PressureCalibrated);
    Serial.println(" hPa");

    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();

    float tempC = ss.getTemp();
    uint16_t capread = ss.touchRead(0);

    //tempCCalibrated = mapfloat(tempC, 18.85, 81.73200, 22.118, 85);
    //capreadCalibrated = mapfloat(capread, 18.85, 81.73200, 22.118, 85);

    Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
    Serial.print("Capacitive: "); Serial.println(capread);
    //Serial.print("Temperature: "); Serial.print(tempCCalibrated); Serial.println("*C");
    //Serial.print("Capacitive: "); Serial.println(capreadCalibrated);

    DynamicJsonDocument doc(64);

    doc["Sensor"] = "BME680";
    doc["Temperature"] = TemperatureCalibrated;
    doc["Humidity"] = HumidityCalibrated;
    doc["Pressure"] = PressureCalibrated;
    doc["Sensor"] = "Capacitive Moisture Sensor";
    doc["SoilHumidity"] = capread;
    doc["SoilTemperature"] = tempC;

    JSONmessage = "";
    serializeJson(doc, JSONmessage);
    JSONmessage.toCharArray(msg, JSONmessage.length() + 1);

    Serial.print("Publish message: ");
    Serial.println(msg);
    Serial.println();
    client.publish("PlantValues", msg);
    client.disconnect();
  }
}
