#include <SPI.h>
#include <LoRa.h>

// Sensor Pins
#define PH_SENSOR A0
#define TDS_SENSOR A1
#define FLOW_SENSOR_PIN 2      // Digital pin for water flow sensor
#define WATER_DEPTH_SENSOR A2  // Analog pin for water depth sensor
#define RAIN_SENSOR_PIN 7
#define SOIL_SENSOR_PIN 8

// LoRa Configuration
#define LORA_CS 10
#define LORA_RST 9
#define LORA_IRQ 2

// Water Flow Variables
volatile unsigned int flowPulseCount = 0;
unsigned long previousMillis = 0;
unsigned long lastFlowTime = 0;
float flowRate = 0.0;

// Flow Sensor Calibration Factor
const float flowCalibrationFactor = 4.5;

void countFlowPulse() {
  flowPulseCount++;
}

void setup() {
  Serial.begin(115200);

  // Initialize Sensor Pins
  pinMode(FLOW_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countFlowPulse, RISING);
  pinMode(PH_SENSOR, INPUT);
  pinMode(TDS_SENSOR, INPUT);
  pinMode(WATER_DEPTH_SENSOR, INPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(SOIL_SENSOR_PIN, INPUT);

  // Initialize LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (true)
      ;
  }
  Serial.println("LoRa initialized.");
}

void loop() {
  sendSensorData();
}

void sendSensorData() {
  // Calculate Flow Rate (Liters per Minute)
  if (millis() - lastFlowTime > 1000) {
    flowRate = (flowPulseCount / flowCalibrationFactor) * 60.0;
    flowPulseCount = 0;
    lastFlowTime = millis();
  }

  // Read Sensor Values
  float pH = readPH();
  float tds = readTDS();
  float flowrate = calculateFlowRate();
  int rain = digitalRead(RAIN_SENSOR_PIN);
  int soilMoisture = digitalRead(SOIL_SENSOR_PIN);
  float waterDepth = readWaterDepth();

  // Create Sensor Data Payload
  String sensorData = String(pH) + "," + String(tds) + "," + String(flowrate, 2) + "," + String(waterDepth) + "," + String(rain) + "," + String(soilMoisture);

  // Send Data via LoRa
  LoRa.beginPacket();
  LoRa.print(sensorData);
  LoRa.endPacket(false);  // Non-blocking mode
  Serial.println("Data sent via LoRa: " + sensorData);

  delay(2000);  // Send data every 2 seconds
}

float readPH() {
  int sensorValue = analogRead(PH_SENSOR);
  float voltage = (sensorValue / 1023.0) * 5.0;  // Assuming 5V reference
  return 7 + ((2.5 - voltage) / 0.18);           // Example calibration
}

float readTDS() {
  int sensorValue = analogRead(TDS_SENSOR);
  float voltage = (sensorValue / 1023.0) * 5.0;  // Assuming 5V reference
  return (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5;
}

float calculateFlowRate() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    flowRate = (flowPulseCount / flowCalibrationFactor);
    flowPulseCount = 0;
    previousMillis = currentMillis;
  }
  return flowRate;
}

float readWaterDepth() {
  int sensorValue = analogRead(WATER_DEPTH_SENSOR);
  float voltage = (sensorValue / 1023.0) * 5.0;  // Assuming 5V reference
  return (voltage / 5.0) * 100.0;                // Assuming sensor gives 0-5V for 0-100cm
}