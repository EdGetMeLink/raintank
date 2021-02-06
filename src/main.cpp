#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <OneWire.h>

#include "ini.h"



#define STACK_PROTECTOR 512 // bytes

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 2

#define VERSION "2.0.2"



/************************* MQTT topics *****************************/
const String raintank_switch = "openhab/raintank/switch";
const String vent_pwm = "openhab/raintank/pwm";
const char *raintank_switch_c = "openhab/raintank/switch";
const char *vent_pwm_c = "openhab/raintank/pwm";

const char *OFF_c = "OFF";
const char *ON_c = "ON";

/****************************** Feeds ***************************************/
const String controller_out = "controller/" + MQTT_NODE + "/signal";
const String available = "controller/" + MQTT_NODE +"/state";
const String raintank_switch_out = "controller/" + MQTT_NODE +"/switch";
const String T_version = "controller/" + MQTT_NODE +"/version";
const String waterlevel = "controller/" + MQTT_NODE +"/waterlevel";
const String temperatur = "controller/" + MQTT_NODE +"/temperatur";

///////////////////////////////////////////////////////////////////////////////
// Set the signal strength and uptime reporting interval in milliseconds
const unsigned long reportInterval = 60000;  // 60 seconds
unsigned long reportTimer = millis();
unsigned int raintank_multiplier = 59;

int old_pwm = 0;
int ventpwm = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
IPAddress ip(10,10,20,7);
IPAddress gw(10,10,20,1);
IPAddress dns(10, 10, 10, 2);
IPAddress sub(255,255,255,192);

int status = WL_IDLE_STATUS;
WiFiUDP udp;

// DS18S20 Temperature chip i/o
OneWire  ds(DSPIN);  // on pin 10 (a 4.7K resistor is necessary)


void measureTemp(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];

  ds.reset_search();
  if ( !ds.search(addr)) {
      Serial.print("No more addresses.\n");
      ds.reset_search();
      return;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;
  float temp = Tc_100 / 100;

  if (SignBit) // If its negative
  {
     Serial.print("-");
  }
  Serial.print(Whole);
  Serial.print(".");
  if (Fract < 10)
  {
     Serial.print("0");
  }
  Serial.print(Fract);

  Serial.print("\n");

 // measure temperature and send to influx and mqtt
  mqttClient.publish(temperatur.c_str(), String(temp).c_str());
  String line = String("temperature,device=wemos,location=garage value=" + String(temp));
  Serial.println(line);
  // send the packet
  Serial.println("Sending UDP packet for Garge Temperature...");
  udp.beginPacket(INFLUXHOST, INFLUXPORT);
  udp.print(line);
  udp.endPacket();


}

void blow_and_measure() 
{
  
  raintank_multiplier += 1;
  // only measure every 180 report intervals
  if (raintank_multiplier >= 60){
    mqttClient.publish(raintank_switch_out.c_str(), "ON");
    raintank_multiplier = 0;
    // 
    digitalWrite(RELAIS_PIN, LOW);
    delay(700);
    digitalWrite(RELAIS_PIN, HIGH);
    delay(1500);
    int level = analogRead(INPUT_PIN);
    mqttClient.publish(waterlevel.c_str(), String(level).c_str());
    mqttClient.publish(raintank_switch_out.c_str(), "OFF");
    String line = String("waterlevel,device=wemos,location=garage value=" + String(level));
    Serial.println(line);
    // send the packet
    Serial.println("Sending UDP packet for Raintank Water Level...");
    udp.beginPacket(INFLUXHOST, INFLUXPORT);
    udp.print(line);
    udp.endPacket();
  }

}



///////////////////////////////////////////////////////////////////////////////
// MQTT Callback
void mqtt_callback(char *topic, byte *payload, unsigned int payloadLength)
{
  payload[payloadLength] = '\0'; 
  if (strcmp(topic, raintank_switch_c) == 0)
  {
    if (strcmp((char *)payload, OFF_c) == 0)
    {
      
    }
    else if (strcmp((char *)payload, ON_c) == 0)
    {
      raintank_multiplier = 60;
      blow_and_measure();      
    }

  }
  
}

///////////////////////////////////////////////////////////////////////////////
// WiFi
void setupWifi()
{
  Serial.println("Connecting to WiFi network: " + String(WLAN_SSID));
  WiFi.hostname(MQTT_NODE.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.config(ip, dns, gw);
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    // Wait 500msec seconds before retrying
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected successfully.");
  Serial.println("assigned IP: " + WiFi.localIP().toString());
}

///////////////////////////////////////////////////////////////////////////////
// OTA
void setupOTA()
{
  // Start up OTA
  // ArduinoOTA.setPort(8266); // Port defaults to 8266
  ArduinoOTA.setHostname(MQTT_NODE.c_str());
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("ESP OTA:  update start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("ESP OTA:  update complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.println("ESP OTA:  ERROR code " + String(error));
    if (error == OTA_AUTH_ERROR)
      Serial.println("ESP OTA:  ERROR - Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("ESP OTA:  ERROR - Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("ESP OTA:  ERROR - Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("ESP OTA:  ERROR - Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("ESP OTA:  ERROR - End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("ESP OTA:  Over the Air firmware update ready");
}

///////////////////////////////////////////////////////////////////////////////
// MQTT
void mqttConnect()
{
  Serial.println("Attempting MQTT connection to broker: " + String(MQTT_SERVER));
  // Attempt to connect to broker, setting last will and testament
  if (mqttClient.connect(MQTT_NODE.c_str(), MQTT_USERNAME, MQTT_PASSWORD, available.c_str(), 1, 1, "OFF"))
  {
    // when connected, record signal strength and reset reporting timer
    String signalStrength = String(WiFi.RSSI());
    reportTimer = millis();
    // publish MQTT discovery topics and device state
    mqttClient.publish(available.c_str(), "ON");
    mqttClient.publish(controller_out.c_str(), signalStrength.c_str());
    mqttClient.subscribe(raintank_switch.c_str());
    // mqttClient.subscribe(vent_pwm.c_str());
    Serial.println("MQTT connected");
  }
  else
  {
    Serial.println("MQTT connection failed, rc=" + String(mqttClient.state()));
    Serial.print("Wifi Status : ");
    Serial.print(WiFi.status());
    if (WiFi.status() != WL_CONNECTED)
    {
      //WiFi.disconnect();
      setupWifi();
    }
    delay(1000);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Setup
void setup()
{
  Serial.begin(9600);
  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.println(F("raintank Wemos D1 Mini v " VERSION));
  setupWifi();
  
    // Start up OTA
  if (otaPassword[0])
  {
    setupOTA();
  }

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqtt_callback);
  // check MQTT connection
  if (!mqttClient.connected())
  {
    mqttConnect();
  }
 
  pinMode(RELAIS_PIN, OUTPUT);
  digitalWrite(RELAIS_PIN, HIGH);
  Serial.println("Raintank v." VERSION " started");
  delay(0);
  Serial.println("Setup complete\n");
}


void loop(){
  mqttClient.loop();
  if (millis() >= (reportTimer + reportInterval))
  {
    reportTimer = millis();
    mqttClient.publish(available.c_str(), "ON");
    mqttClient.publish(T_version.c_str(), VERSION);
    mqttClient.publish(controller_out.c_str(), String(WiFi.RSSI()).c_str());
    blow_and_measure();
    measureTemp();
  }
  ArduinoOTA.handle();

}

