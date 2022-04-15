#include "WiFi.h"
#include "PubSubClient.h"
#include <PZEM004Tv30.h>
//#define ESP32
/* Hardware Serial2 is only available on certain boards.
   For example the Arduino MEGA 2560
*/
#if defined(ESP32)
PZEM004Tv30 pzem(Serial2, 13, 15);
#else
PZEM004Tv30 pzem(Serial2);
#endif
//#define MQTT_VERSION MQTT_VERSION_3_1

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const char* ssid = " ";   // Your_SSID 
const char* password = " ";  // Your_Password 

const char *mqttServer = " "; // host ip
const char *clientID = "101@doesnotmatter";
const char *topic = "0";
const uint16_t mqttPort = 3004;
const char* mqttUser = " "; 
const char* mqttPassword = " ";


void setup()
{
  // Set software serial baudrate to 115200;
  Serial.begin(115200);

  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("...");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");
  // Serial.println(WiFi.localIP());

  // connecting to a mqtt broker
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  //  client.setKeepAlive (30);
  //  client.setSocketTimeout (30);

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

void loop() {
  if (!client.connected())
    reconnect();
  client.loop();

  Serial.print("Custom Address:");
  Serial.println(pzem.readAddress(), HEX);

  // Read the data from the sensor
  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  // Check if the data is valid
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

    // Print the values to the Serial console
    Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
    Serial.print("Current: ");      Serial.print(current, 5);      Serial.println("A");
    Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
    Serial.print("Energy: ");       Serial.print(energy, 3);     Serial.println("kWh");
    Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
    Serial.print("PF: ");           Serial.println(pf);

    // define topics for each value to send to the claud
    const char *voltage_topic = "0";
    const char *amperage_topic = "1";
    const char *power_topic = "2";
    const char *energy_topic = "3";
    const char *frequency_topic = "4";
    const char *power_facrot_topic = "5";
    
    // convert float values into char[]
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
    client.publish("0", send_voltage);
    client.publish("1", send_current);
    client.publish("2", send_power);
    client.publish("3", send_energy);
    client.publish("4", send_frequency);
    client.publish("5", send_pf);
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
