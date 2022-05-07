#include "WiFi.h"
#include "PubSubClient.h"
#include "PZEM004Tv30.h"

#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 13, 15);
#else
PZEM004Tv30 pzem(Serial2);
#endif

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const char* ssid = "";      // SSID
const char* password = "";  // Password

/*
  clientID = "101@ESP32-1"; // ESP #1
  clientID = "115@ESP32-2"; // ESP #2
  clientID = "122@ESP32-3"; // ESP #3
*/
const char *mqttServer = ""; // Host IP
const char *clientID = "101@ESP32-1";
const uint16_t mqttPort = 3004;
const char* mqttUser = "";
const char* mqttPassword = "";

// define topics for each value to send to the claud
const char *voltage_topic     = "0";
const char *current_topic     = "1";
const char *power_topic       = "2";
const char *energy_topic      = "3";
const char *frequency_topic   = "4";
const char *pf_topic          = "5";

void setup()
{
  // Set software serial baudrate to 115200;
  Serial.begin(115200);

  // connecting to the WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("...");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");

  // connecting to a mqtt broker
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Serial.println("Connecting to MQTT Broker...");
  while (!client.connected()) {
    // Attempt to connect
    if (client.connect(clientID, mqttUser, mqttPassword)) {
      Serial.println("Sucessfully Connected to MQTT Broker.");
    }
    else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      Serial.println("");
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected())
    reconnect();
  client.loop();

  Serial.print("Custom Address:");
  Serial.println(pzem.readAddress(), HEX);

  // Read data from the sensors
  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  // Check data validity
  if (isnan(voltage)) {
    Serial.println("Error reading voltage");
  } else if (isnan(current)) {
    Serial.println("Error reading current");
  } else if (isnan(power)) {
    Serial.println("Error reading power");
  } else if (isnan(energy)) {
    Serial.println("Error reading energy");
  } else if (isnan(frequency)) {
    Serial.println("Error reading frequency");
  } else if (isnan(pf)) {
    Serial.println("Error reading power factor");
  } else {
    char send_voltage[20];
    dtostrf(voltage, 2, 3, send_voltage);
    char send_current[20];
    dtostrf(current, 2, 3, send_current);
    char send_power[20];
    dtostrf(power, 2, 3, send_power);
    char send_energy[20];
    dtostrf(energy, 2, 3, send_energy);
    char send_frequency[20];
    dtostrf(frequency, 2, 3, send_frequency);
    char send_pf[20];
    dtostrf(pf, 2, 3, send_pf);

    // publish values to corresponding topics
    client.publish(voltage_topic, send_voltage);
    client.publish(current_topic, send_current);
    client.publish(power_topic, send_power);
    client.publish(energy_topic, send_energy);
    client.publish(frequency_topic, send_frequency);
    client.publish(pf_topic, send_pf);

    // Print values to the Serial console
    Serial.print("Voltage: ");    Serial.print(voltage);      Serial.println("V");
    Serial.print("Current: ");    Serial.print(current, 5);   Serial.println("A");
    Serial.print("Power: ");      Serial.print(power);        Serial.println("W");
    Serial.print("Energy: ");     Serial.print(energy, 3);    Serial.println("kWh");
    Serial.print("Frequency: ");  Serial.print(frequency, 1); Serial.println("Hz");
    Serial.print("PF: ");         Serial.println(pf);

  }
  Serial.println();
  delay(2000);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT Broker..");
    // Attempt to connect
    if (client.connect(clientID, mqttUser, mqttPassword)) {
      Serial.println("Sucessfully Connected to MQTT Broker.");
    }
    else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      Serial.println("");
      delay(2000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("New Message at topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("------------------------------------");
}