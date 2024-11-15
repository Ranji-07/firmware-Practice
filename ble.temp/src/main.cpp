#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>  

#define DHTPIN 4    
#define DHTTYPE DHT11    // DHT11 sensor

// Define BLE service and characteristics
BLEService EVMService("181A");
BLEUnsignedIntCharacteristic EVMLevelChar_temperature("2A6E", BLERead|BLENotify);
BLEUnsignedIntCharacteristic EVMLevelChar_humidity("2A6F", BLERead|BLENotify); // Added humidity

DHT dht(DHTPIN, DHTTYPE);
long previousMillis = 0;
const unsigned long updateInterval = 2000;

int targetRSSI = WiFi.RSSI();
bool targetNetworkFound = false;
bool isConnectedToWiFi = false;
bool isConnectedToBLE = false;
bool isConnected = false;

float t = 0.0;
float h = 0.0;

// WiFi credentials
const char* targetSSID = "Smilepls";
const char* targetPassword = "10203040";
const char* mqttServer = "restdev.boodskap.io";  // Change this to your MQTT broker
const int mqttPort = 1883;  // MQTT default port
const char* mqttTopic = "/P85TRMALUS6JJJI7/AA0012/pub/0";  // MQTT topic to publish data


const String deviceId = "AA0012";  
const String apiKey = "DEVYaJ6fOGJj4pRRlF";     

WiFiClient espClient;
PubSubClient mqttClient(espClient);  // Create MQTT client object

void scanNetworks();
bool connectToBLE();
bool connectToWiFi(const String& targetSSID, const String& targetpassword);
void weatherData();
void sendToWiFi();
void sendToBLE();
void reconnectMQTT();

void setup() 
{
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  dht.begin();  
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
}

void loop()
{
  BLEDevice central = BLE.central();
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;
    weatherData();
    
    if (isConnectedToWiFi) {
      if (!mqttClient.connected()) reconnectMQTT();
      mqttClient.loop();
      sendToWiFi();
    } else if (isConnectedToBLE && central) {
      sendToBLE();
    }
  }

  if (!isConnectedToWiFi) {
    scanNetworks();
    if (central && !isConnectedToBLE) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());
      isConnectedToBLE = true;
      WiFi.disconnect();
    } else if (!central && isConnectedToBLE) {
      Serial.println("Disconnected from central");
      isConnectedToBLE = false;
      isConnectedToWiFi = connectToWiFi(targetSSID, targetPassword);
    }
  }
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

  if (targetNetworkFound) 
  {
    isConnected = connectToWiFi(targetSSID, targetPassword); 
  }
}

bool connectToWiFi(const String& targetSSID, const String& targetpassword) 
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(targetSSID.c_str(), targetpassword.c_str());
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) 
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

bool connectToBLE()
{
  BLE.setLocalName("ESS MONITOR");  
  BLE.setDeviceName("ESS DEVICE");  // Environmental Sensing Service
  BLE.setAppearance(0x0300);        // Set appearance to a generic sensor
  EVMService.addCharacteristic(EVMLevelChar_temperature);           // ESS
  EVMService.addCharacteristic(EVMLevelChar_humidity); // Added humidity characteristic
  
  BLE.addService(EVMService);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  return true;
} 

void weatherData()
{
  h = dht.readHumidity();
  t = dht.readTemperature();

  if (isnan(h) || isnan(t)) 
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  else 
  {
    Serial.println("Sensor readings are valid.");
    Serial.println("Temperature: " + String(t) + "°C");
    Serial.println("Humidity: " + String(h) + "%");
  } 
}

void sendToWiFi()
{
  if (!mqttClient.connected()) 
  {
    reconnectMQTT();
  }

  // Create the JSON payload
  String payload = "{\"device_id\":\"" + String(deviceId) + "\",\"api_key\":\"" + String(apiKey) + "\",\"temperature\":" + String(t) + ",\"humidity\":" + String(h) + "}";

  // Publish the payload to the MQTT topic
  if (mqttClient.publish(mqttTopic, payload.c_str())) 
  {
    Serial.println("Data sent successfully over MQTT.");
  } 
  else 
  {
    Serial.println("Error in sending data via MQTT.");
  }
}

void sendToBLE()
{
  EVMLevelChar_temperature.writeValue(dht.readTemperature() * 100); 
  EVMLevelChar_humidity.writeValue(dht.readHumidity() * 100);    
  Serial.println("Sent data over BLE.");
}

void reconnectMQTT()
{
  // Loop until we're reconnected to the MQTT broker
  while (!mqttClient.connected()) 
  {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    delay(100);
  }
}
