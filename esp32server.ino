#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>

// LoRa Configuration
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 26

// WiFi Credentials
const char* ssid = "Divyansh Jha";
const char* password = "Divyansh";

// MQTT Broker
const char* mqtt_server = "broker.emqx.io";
const char* mqtt_topic = "Multisensor";

// MQTT Subscription Topic
const char* instruction_topic = "PUMP/HUMID";

WiFiClient espClient;
PubSubClient client(espClient);

// DWIN Display UART
#define TX2_PIN 17
#define RX2_PIN 16
HardwareSerial DWIN(2);

// LoRa Data Buffer
String receivedData;

// Reconnection Variables
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 2000;  // 2 seconds interval

// DWIN Buffer
uint8_t Buffer[9];

void setup() {
  Serial.begin(115200);
  DWIN.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
  }
  Serial.println("LoRa initialized.");

  setupWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);  // Set callback for MQTT messages
  client.setKeepAlive(60);           // Keep-alive interval
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) setupWiFi();
  if (!client.connected()) {
    connectToMQTT();
  } else {
    client.loop();  // Keep MQTT connection alive
  }

  pump_control();     // Handle DWIN instructions
  processLoRaData();  // Handle LoRa received data
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
}

// MQTT Callback Function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("Received from MQTT: " + message);

  if (String(topic) == instruction_topic) {
    processInstruction(message);
  }
}

// Process Instruction and Send via LoRa
void processInstruction(String instruction) {
  // Validate instruction format
  if (instruction.startsWith("PUMP1,") || instruction.startsWith("PUMP2,") || instruction.startsWith("PUMP3,") || instruction.startsWith("HUMIDIFIER")) {
    Serial.println("Forwarding to LoRa: " + instruction);
    if (sendDataToLoRa(instruction)) {
      Serial.println("Instruction sent to LoRa successfully.");
    } else {
      Serial.println("Failed to send instruction to LoRa!");
    }
  } else {
    Serial.println("Invalid instruction format.");
  }
}

void connectToMQTT() {
  String clientId = "ESP32Client-" + String(random(0xffff), HEX);
  Serial.print("Connecting to MQTT...");
  if (client.connect(clientId.c_str())) {
    Serial.println("connected");
    client.subscribe(instruction_topic);  // Subscribe to instruction topic
    Serial.println("Subscribed to topic: " + String(instruction_topic));
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" retrying...");
  }
}

void processLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    Serial.println("Received from LoRa: " + receivedData);
    // Send data to MQTT broker
    sendDataToMQTT(receivedData);

    // Send data to DWIN Display
    sendDataToDWIN(receivedData);
  }
}

void sendDataToMQTT(String data) {
  // Parse received data (-4.04,5293.26,0.00,89.15,1,1)
  int delimiter1 = data.indexOf(',');
  int delimiter2 = data.indexOf(',', delimiter1 + 1);
  int delimiter3 = data.indexOf(',', delimiter2 + 1);
  int delimiter4 = data.indexOf(',', delimiter3 + 1);
  int delimiter5 = data.indexOf(',', delimiter4 + 1);

  float pH = data.substring(0, delimiter1).toFloat();
  float tds = data.substring(delimiter1 + 1, delimiter2).toFloat();
  float flowrate = data.substring(delimiter2 + 1, delimiter3).toFloat();
  float waterDepth = data.substring(delimiter3 + 1, delimiter4).toFloat();
  int rain = data.substring(delimiter4 + 1, delimiter5).toInt();
  int soilMoisture = data.substring(delimiter5 + 1).toInt();

  // Format payload
  String payload = "{\"pH\":";
  payload += String(pH, 2);  // Two decimal places
  payload += ", \"TDS\":";
  payload += String(tds, 2);  // Two decimal places
  payload += ", \"WFS\":";
  payload += String(flowrate, 2);  // Two decimal places
  payload += ", \"WDS\":";
  payload += String(waterDepth, 2);  // Two decimal places
  payload += ", \"RM\":";
  payload += String(rain);
  payload += ", \"SM\":";
  payload += String(soilMoisture);
  payload += "}";

  // Publish to MQTT
  if (millis() - lastPublishTime >= publishInterval) {
    if (client.publish(mqtt_topic, payload.c_str(), true)) {  // QoS 1
      Serial.println("Data sent to MQTT: " + payload);
    } else {
      Serial.print("Failed to send data to MQTT. Error code: ");
      Serial.println(client.state());
    }
    lastPublishTime = millis();
  }
}

void sendDataToDWIN(String data) {
  int delimiter1 = data.indexOf(',');
  int delimiter2 = data.indexOf(',', delimiter1 + 1);
  int delimiter3 = data.indexOf(',', delimiter2 + 1);
  int delimiter4 = data.indexOf(',', delimiter3 + 1);
  int delimiter5 = data.indexOf(',', delimiter4 + 1);

  int pH = data.substring(0, delimiter1).toInt();
  int tds = data.substring(delimiter1 + 1, delimiter2).toInt();
  int flowrate = data.substring(delimiter2 + 1, delimiter3).toInt();
  int waterDepth = data.substring(delimiter3 + 1, delimiter4).toInt();
  int rain = data.substring(delimiter4 + 1, delimiter5).toInt();
  int soilMoisture = data.substring(delimiter5 + 1).toInt();

  // DWIN commands as before
  unsigned char cmd[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00 };

  // Send pH
  cmd[4] = 0x51;
  cmd[6] = highByte(pH);
  cmd[7] = lowByte(pH);
  DWIN.write(cmd, 8);

  // Send TDS
  cmd[4] = 0x52;
  cmd[6] = highByte(tds);
  cmd[7] = lowByte(tds);
  DWIN.write(cmd, 8);

  // Send Flowrate
  cmd[4] = 0x53;
  cmd[6] = highByte(flowrate);
  cmd[7] = lowByte(flowrate);
  DWIN.write(cmd, 8);

  // Send Water Depth
  cmd[4] = 0x54;
  cmd[6] = highByte(waterDepth);
  cmd[7] = lowByte(waterDepth);
  DWIN.write(cmd, 8);

  // Send Rain
  cmd[4] = 0x59;
  cmd[6] = highByte(rain);
  cmd[7] = lowByte(rain);
  DWIN.write(cmd, 8);

  // Send Soil Moisture
  cmd[4] = 0x60;
  cmd[6] = highByte(soilMoisture);
  cmd[7] = lowByte(soilMoisture);
  DWIN.write(cmd, 8);

  Serial.println("Data sent to DWIN.");
}

void pump_control() {
  if (DWIN.available()) {
    for (int i = 0; i < 9; i++) {
      Buffer[i] = DWIN.read();  // Store entire frame in buffer
    }

    if (Buffer[0] == 0x5A) {  // Check for valid frame
      String controlMessage = "";
      String mqttMessage = "";

      switch (Buffer[4]) {
        case 0x55:  // PUMP1
          controlMessage = (Buffer[8] == 1) ? "PUMP1,ON" : "PUMP1,OFF";
          break;

        case 0x56:  // PUMP2
          controlMessage = (Buffer[8] == 1) ? "PUMP2,ON" : "PUMP2,OFF";
          break;

        case 0x57:  // PUMP3
          controlMessage = (Buffer[8] == 1) ? "PUMP3,ON" : "PUMP3,OFF";
          break;

        case 0x58:  // HUMIDIFIER
          controlMessage = (Buffer[8] == 1) ? "HUMIDIFIER,ON" : "HUMIDIFIER,OFF";
          break;

        default:
          return;
      }

      // Set mqttMessage to controlMessage
      mqttMessage = controlMessage;  // Update the mqttMessage with controlMessage

      // Send control message to LoRa client
      if (sendDataToLoRa(controlMessage)) {
        Serial.println("Sent to LoRa: " + controlMessage);
      } else {
        Serial.println("Failed to send instruction to LoRa!");
      }

      // Publish to MQTT
      if (client.publish(mqtt_topic, mqttMessage.c_str())) {
        Serial.println("Published to MQTT: " + mqttMessage);
      } else {
        Serial.println("Failed to publish MQTT message");
      }
    }
  }
}

bool sendDataToLoRa(String data) {
  LoRa.beginPacket();
  LoRa.print(data);
  return LoRa.endPacket() == 1;
}