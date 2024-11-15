#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>   
#include <Adafruit_Sensor.h> 
#include <DHT.h> 
#include <ArduinoBLE.h>
#include <SPI.h>
#include <PubSubClient.h>

#define IR_SENSOR 13
#define LED 2
#define DHTPIN 4
#define ANEMOMETER_PIN 5
#define DHTTYPE DHT11
#define RAIN_SENSOR_PIN A0
#define RELAY_PIN 12
#define LIGHT_SENSOR_PIN A10

BLEService EVMService("181A");
BLEUnsignedIntCharacteristic EVMLevelChar_temperature("2A6E", BLERead|BLENotify);
BLEUnsignedIntCharacteristic EVMLevelChar_humidity("2A6F", BLERead|BLENotify); // Added humidity
BLEUnsignedIntCharacteristic EVMLevelChar_windSpeed("2A6D", BLERead|BLENotify); // Added wind speed characteristic
BLEUnsignedIntCharacteristic EVMLevelChar_rainSensor("2A70", BLERead|BLENotify); // Added rain sensor characteristic
BLEUnsignedIntCharacteristic EVMLevelChar_lightSensor("2A71", BLERead|BLENotify); // Added light sensor characteristic
BLEUnsignedIntCharacteristic EVMLevelChar_status("2A72", BLERead|BLENotify); // Added status characteristic


DHT dht(DHTPIN, DHTTYPE);
long previousMillis = 0;
const unsigned long updateInterval = 2000;

const char* targetSSID = "Smilepls";
const char* targetPassword = "10203040";

int targetRSSI = WiFi.RSSI();
int sensorValue = digitalRead(IR_SENSOR);
bool targetNetworkFound = false;
bool isConnectedToWiFi = false;
bool isConnectedToBLE = false;
bool isConnected = false;
bool objectDetected = false;
float t = 0.0;
float h = 0.0;
float f = 0.0;
volatile unsigned long windSpeedPulseCount = 0;
int rainSensorValue;
int lightValue ;
String status;

const char* mqttServer = "broker.hivemq.com";  // Change this to your MQTT broker
const int mqttPort = 1883;  // MQTT default port
const char* mqttUser = "";  // Optional: Add username for the MQTT broker
const char* mqttTopic = "home/esp32/env"; 
const String deviceId = "your_device_id";  
const String apiKey = "your_api_key";

WiFiClient espClient;
PubSubClient mqttClient(espClient); 

String getChipID();
void scanNetworks();
void irSensor();
void weatherData();
void led();
float getWindSpeed(); 
void rainSensor();
int LightSensor();
void sendJson(int numNetworks, const String& targetSSID, int targetRSSI, 
         bool targetNetworkFound, bool connectToWiFi, const String& SSID, const String& password,
        float h, float t, float f, String status);
bool connectToWiFi(const String& targetSSID, const String& targetpassword);
bool connectToBLE();
void sendToWiFi();
void sendToBLE();
void reconnectMQTT();
void bleCentral();

void setup()
{
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  dht.begin();
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), RISING);

  pinMode(IR_SENSOR, INPUT);
  pinMode(LED,OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);  
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  while (!Serial);
  pinMode(2, OUTPUT);

  if (!BLE.begin()) 
  {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Initial connection attempts
  isConnectedToWiFi = connectToWiFi(targetSSID, targetPassword);
  if (!isConnectedToWiFi) 
  {
    connectToBLE(); // If WiFi fails, try BLE
  }

  mqttClient.setServer(mqttServer, mqttPort);  // Setup MQTT server
  delay(1000);
  EVMLevelChar_temperature.writeValue(0); 
  EVMLevelChar_humidity.writeValue(0);
  EVMLevelChar_windSpeed.writeValue(0);
  EVMLevelChar_rainSensor.writeValue(0); 
  EVMLevelChar_lightSensor.writeValue(0);  
}

void loop()
{
  scanNetworks();
  bleCentral(); // Check for BLE connections

  if (isConnectedToWiFi) 
  {
    weatherData();
    irSensor();
    led();
    float windSpeed = getWindSpeed();
    rainSensor();
    lightValue = LightSensor(); // Update lightValue
    sendToWiFi();
    delay(5000); 
  } 
  else if (isConnectedToBLE) 
  {
    weatherData(); // Read sensor data for BLE as well
    float windSpeed = getWindSpeed();
    rainSensor();
    lightValue = LightSensor(); // Update lightValue
    sendToBLE();
    delay(5000); 
  }
}

String getChipID() 
{
  uint64_t chipid = ESP.getEfuseMac();
  String chipID = String(chipid, HEX);
  chipID.toUpperCase();
  return chipID;
}

void scanNetworks() 
{
  int numNetworks = WiFi.scanNetworks();
  if (numNetworks == 0) 
  {
    return; 
  }

  for (int i = 0; i < numNetworks; ++i) 
  {
    if (WiFi.SSID(i) == targetSSID) 
    {
      targetRSSI = WiFi.RSSI(i);
      targetNetworkFound = true;
      break; 
    } 
  }

  bool isConnected = false; // Renamed the variable
  const String& currentSSID = WiFi.SSID(); 

  if (targetNetworkFound) 
  {
    isConnected = connectToWiFi(targetSSID, targetPassword); 
  }

  sendJson(numNetworks, targetSSID, targetRSSI, 
          targetNetworkFound, isConnected, (targetNetworkFound ? targetPassword : ""),
           h, t, f, "status");
}

bool connectToWiFi(const String& targetSSID, const String& targetpassword) 
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(targetSSID.c_str(), targetpassword.c_str());
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 1000) 
  {
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) 
  {
    Serial.println("Connected to WiFi!");
    reconnectMQTT();  // Connect to MQTT when WiFi is connected
  }
  return WiFi.status() == WL_CONNECTED;
}

void weatherData()
{
    h = dht.readHumidity();
    t = dht.readTemperature();
    f = dht.readTemperature(true);  
}

void irSensor() 
{
  objectDetected = digitalRead(IR_SENSOR) == LOW; 
  digitalWrite(LED, objectDetected);
}

float getWindSpeed() 
{
  float windSpeed = (float)windSpeedPulseCount * 2.4; 
  windSpeedPulseCount++;
  windSpeedPulseCount = 0; 
  return windSpeed;
}

void rainSensor() 
{
  rainSensorValue = analogRead(RAIN_SENSOR_PIN); 
  
  if (rainSensorValue < 500) 
{
    status = "Rain Alert!";
} 
else 
{
    status = "No Rain";
}
}

int LightSensor() 
{
  int lightSensorValue = analogRead(LIGHT_SENSOR_PIN); 
  return lightSensorValue;
}

void sendJson(int numNetworks, const String& targetSSID, int targetRSSI, 
         bool targetNetworkFound,bool connectToWiFi,const String& SSID, const String& password,
        float h, float t, float f,String status) 
{
  JsonDocument doc;
  doc["chipID"] = getChipID();
  doc["numNetworks"] = numNetworks;
  doc["targetNetworkFound"] = targetNetworkFound;
  if (targetNetworkFound) 
  {
    doc["targetSSID"] = targetSSID;
    doc["targetRSSI"] = targetRSSI;
    doc["connectToWiFi"] = isConnected;
    doc["ip_address"] = WiFi.localIP().toString();
  }
  doc["objectDetected"] = objectDetected;
  doc["t"] = t;
  doc["h"] = h;
  doc["f"] = f;
  doc["windSpeed"] = getWindSpeed();
  doc["rainSensorValue"] = rainSensorValue;

  String jsonString;
  serializeJsonPretty(doc, jsonString);
  Serial.println(jsonString);
  Serial.println();
}

bool connectToBLE()
{
  BLE.setLocalName("ESS MONITOR");  
  BLE.setDeviceName("ESS DEVICE");  // Environmental Sensing Service
  BLE.setAppearance(0x0300);        // Set appearance to a generic sensor

  BLE.addService(EVMService);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  return true;
} 

void sendToWiFi()
{
  if (!mqttClient.connected()) 
  {
    reconnectMQTT();
  }

  // Create the JSON payload
 String payload = "{\"device_id\":\"" + String(deviceId) + "\",\"api_key\":\"" + String(apiKey) + "\",\"temperature\":" + String(t) + ",\"humidity\":" + String(h) + ",\"windSpeed\":" + String(getWindSpeed()) + ",\"rainSensorValue\":" + String(rainSensorValue) + ",\"lightValue\":" + String(lightValue) + "}";
}

void sendToBLE()
{
 EVMLevelChar_temperature.writeValue((int)(dht.readTemperature() * 100));  // Cast float to int
EVMLevelChar_humidity.writeValue((int)(dht.readHumidity() * 100));        // Cast float to int
EVMLevelChar_windSpeed.writeValue((int)(getWindSpeed() * 100));  
  EVMLevelChar_rainSensor.writeValue(rainSensorValue); 
  EVMLevelChar_lightSensor.writeValue(lightValue); 
  Serial.println("Sent data over BLE.");
}

void reconnectMQTT()
{
    if (!mqttClient.connected()) 
    {
        if (mqttClient.connect("ESP32Client")) 
        {
            Serial.println("Connected to MQTT");
        } 
        else 
        {
            Serial.print("MQTT connection failed, rc=");
            Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

void bleCentral()
{
  BLEDevice central = BLE.central();
  if (central) 
  {
    while (central.connected())
    isConnectedToBLE = true;
    isConnectedToWiFi = false;
    WiFi.disconnect();
  }
  else
  {
    isConnectedToBLE = false;
    isConnectedToWiFi = true;
  }
}
