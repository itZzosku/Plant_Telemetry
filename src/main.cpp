/*
 Basic ESP32 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP32 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP32 Arduino:
  - https://github.com/espressif/arduino-esp32

 #include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

void setup() {
  Serial.begin(115200);

  Serial.println("seesaw Soil Sensor example!");
  
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
}

void loop() {
  float tempC = ss.getTemp();
  uint16_t capread = ss.touchRead(0);

  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(capread);
  delay(100);
} 

*/

#include <WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME680.h>

#include <ArduinoJson.h>
#include "Adafruit_seesaw.h"

#include <credentials.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED 13
#endif

#define I2C_SDA 23
#define I2C_SCL 22

Adafruit_seesaw ss;

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

    float tempC = ss.getTemp();
    uint16_t capread = ss.touchRead(0);

    //tempCCalibrated = mapfloat(tempC, 18.85, 81.73200, 22.118, 85);
    //capreadCalibrated = mapfloat(capread, 18.85, 81.73200, 22.118, 85);

    Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
    Serial.print("Capacitive: "); Serial.println(capread);
    //Serial.print("Temperature: "); Serial.print(tempCCalibrated); Serial.println("*C");
    //Serial.print("Capacitive: "); Serial.println(capreadCalibrated);

    DynamicJsonDocument doc(64);

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
