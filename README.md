SAOS is an IoT-based system using sensors, Arduino Nano, L298 motor drivers, and LoRa modules to monitor key agricultural parameters.
It optimizes irrigation, tracks soil and water quality, conserves resources, and ensures optimal crop health.

Through this Repository, everyone can make and utilise this educational kit.

arduinonano_pump_humidifier_data.ino - This file is used in the Client Node to control water pumps and the humidifier. Instructions are received from the ESP32 server via the LoRa communication protocol and executed on the Client Node.
arduinonano_sensor_data.ino - This file is used in the Client Node to instruct sensors for sensing data. Sensed data is sent to ESP32 Server using LoRa Communication Protocol.
esp32server.ino - Server file designed to work on the ESP32 Dev Module via the Arduino IDE, implemented for the Smart Agriculture Optimization System (SAOS). Communication protocols utilized: WiFi, MQTT, LoRa, and UART. The ESP32 server receives sensor data from the Arduino Nano (Client Node) via the LoRa communication protocol and transmits this data to the HMI DWIN Display via UART and to Node-RED via MQTT. The ESP32 server receives instructions from the HMI DWIN Display through UART and updates the status on Node-RED. Additionally, it receives instructions from Node-RED through MQTT and forwards these to the Client Node via LoRa.

for more Info:
about Authors - Designers.docx - For Contact and Communication purpose with Authors and Designers of SAOS.

Video demonstration of SAOS: https://drive.google.com/file/d/1j4aMBhUMu1RCyj2zIHCP-n9icUefnGQr/view?usp=drive_link
