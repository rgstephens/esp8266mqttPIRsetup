/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *  
 *  Configurable MQTT topics
 *    esp8266/config/delay       // main loop delay in milleseconds
 *    esp8266/config/ipinterval  // number of intervals between sending IP address, zero to never send
 *    
 *  Uses the WiFiManager: https://github.com/tzapu/WiFiManager
 *
 */
 
#include <FS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#define PROG_NAME "esp8266mqttPIR, "
#define PROG_VERS "v0.34"

const char* ssid     = "nworks24";
const char* password = "golfbgc1";

char mqtt_server[40] = "15.1.4.4";
char mqtt_port[6] = "8883";

//#define MQTT_SERVER  "15.1.4.4"
//#define MQTT_PORT    8883
#define MQTT_TOPICBASE "esp8266/"

//flag for saving data
bool shouldSaveConfig = false;

int pirPin = D6;
int pirLastState = 0;

// Sleep time - 2s = 2000, 1m = 60000, 5m = 300000, 15m = 900000
int delayMS = 2000;

// number of intervals to send IP address, 0 = never (only on startup)
int ipInterval = 90;
int intervalCount = 0;

String strIp = "";

// Use WiFiClient class to create TCP connections
WiFiClient client;
PubSubClient mqttClient(client);

void setup() {
  Serial.begin(115200);
  
  Serial.print(F("Started, "));
  Serial.print(PROG_NAME);
  Serial.println(PROG_VERS);

  //Serial.println("formatting FS...");
  //SPIFFS.format();
  /*
  Serial.println("mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
  } else {
    Serial.println("could not mount FS, formatting...");
    SPIFFS.format();
    if (SPIFFS.begin()) {
      Serial.println("mounted file system");
    }
  }
*/
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pirPin, INPUT);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  
  WiFiManager wifiManager;
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,1,1,1), IPAddress(10,1,1,200), IPAddress(255,255,255,0));
  
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //wifiManager.autoConnect("esp8266");
  wifiManager.startConfigPortal("esp8266");

  Serial.print("mqtt_port: ");
  Serial.println(custom_mqtt_port.getValue());
  
  // We start by connecting to a WiFi network
  /*
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
*/
  strIp = IpAddress2String(WiFi.localIP());
  Serial.print("WiFi connected, IP: ");  
  Serial.println(WiFi.localIP());

  int mqttPort;
  mqttPort = atoi(mqtt_port);

  mqttClient.setServer(mqtt_server, mqttPort);
  //mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);
  
  Publish((char *)"proginfo", PROG_NAME);
  sendIP();
  PublishInt((char *)MQTT_TOPICBASE "delay", delayMS);
  PublishInt((char *)MQTT_TOPICBASE "ipinterval", ipInterval);
  
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
}

void loop() {
  intervalCount++;
  mqttClient.loop();
  // Make sure we're connected to the MQTT broker
  if (!mqttClient.connected()) {
    Serial.println(">> mqtt connect <<");
    //Serial.println((char *)MQTT_TOPICBASE "name");
    //mqttClient.setServer(mqtt_server, mqttPort);
    mqttClient.connect((char *)MQTT_TOPICBASE "name");

    if (mqttClient.connected()) {
      Serial.print("subscribe: ");
      Serial.println(MQTT_TOPICBASE "config/#");
      boolean status = mqttClient.subscribe(MQTT_TOPICBASE "config/#");  // Allow bootup config fetching using MQTT persist flag!
    }
  }
  //while (!mqttClient.connected()) {
  //}
  mqttClient.loop();
  if (mqttClient.connected()) {
    if (ipInterval > 0 && intervalCount >= ipInterval) {
      intervalCount = 0;
      sendIP();
      delay(500);
    }
  }
  // check for PIR change
  int pirValue = digitalRead(pirPin);
  if (pirLastState != pirValue) {
    if (pirValue == 1) {
      digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    } else {
      digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    }
    pirLastState = pirValue;
    Serial.print("PIR State Change, Value: ");
    Serial.println(pirValue);
    if (mqttClient.connected()) {
      PublishInt((char *)MQTT_TOPICBASE "motion", pirValue);
    }
  }
  mqttClient.loop();
  delay(delayMS);
}

void callback(char* topic, byte* payload, unsigned int length)
{
  payload[length] = 0;    // Hack to be able to use this as a char string.

  if (strstr(topic, MQTT_TOPICBASE "config/"))
  {
    if (strstr(topic, "delay")) {
      int oldDelayMS = delayMS;
      delayMS = atoi((const char *)payload);
      if (delayMS != oldDelayMS) {
        Serial.print("setting delay to ");
        Serial.println(delayMS);
        PublishInt((char *)MQTT_TOPICBASE "delay", delayMS);
      }
    } else if (strstr(topic, "ipinterval")) {
      int oldIpInterval = ipInterval;
      ipInterval = atoi((const char *)payload);
      if (ipInterval != oldIpInterval) {
        Serial.print("setting ipInterval to ");
        Serial.println(ipInterval);
        PublishInt((char *)MQTT_TOPICBASE "ipinterval", ipInterval);
      }
    } else {
      Serial.print("Unknown topic received: ");
      Serial.println(topic);
    }
  }
}

void sendIP() {
    Serial.print("mqtt send IP addr: ");
    Serial.println(WiFi.localIP());
    mqttClient.publish((char *)MQTT_TOPICBASE "ip", strIp.c_str());
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

void Publish(char *Topic, char *Message) {
  char TopicBase[80] = MQTT_TOPICBASE;

  strcat(TopicBase, Topic);
  mqttClient.publish(TopicBase, Message);
}

void PublishInt(char *Topic, int Value) {
  char TopicBase[80] = MQTT_TOPICBASE;
  char Message[10] = "NULL";

  if (!isnan(Value))
    itoa(Value, Message, 10);

  mqttClient.publish(Topic, Message);
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


