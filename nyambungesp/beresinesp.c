#include <WiFi.h>
#include <WiFiUdp.h>

// ================= WIFI CONFIG =================
const char* ssid = "RobotAP";
const char* password = "12345678";

// ================= UDP CONFIG ==================
const int udpPort = 50002;
WiFiUDP Udp;

// ================= UART CONFIG =================
#define RXD2 16
#define TXD2 17
#define UART_BAUD 115200

// ================= DATA STRUCT =================
struct ControlData {
  float servoAngle; 
  uint16_t motorPWM;     // 0–999
};
struct SensorData {
  float distance;
  bool obstacle;
  uint16_t motorSpeed;
  float servoAngle;
};

ControlData control = {90, 0};
SensorData sensorData = {0, false, 0, 90};

// Buffer RX laptop
uint16_t udpBuffer[255];

// Buffer UART dari STM32
uint16_t uartBuffer[255];
uint16_t uartIndex = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);
  Serial.println("ESP32 AP Mode aktif");

  Udp.begin(udpPort);
}

// ================= MAIN LOOP =================
void loop() {
  receiveFromLaptop();
  receiveFromSTM32();

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {
    sendToSTM32();
    lastSend = millis();
  }
}

// ====================================================
//     RECEIVE FROM LAPTOP (UDP)
// ====================================================
void receiveFromLaptop() {
  int packetSize = Udp.parsePacket();
  if (!packetSize) return;

  // Baca ke char buffer dulu
  char temp[255];
  int len = Udp.read(temp, 255);
  temp[len] = 0;

  // Copy ke buffer uint16_t
  for (int i = 0; i < len; i++) {
    udpBuffer[i] = (uint16_t)temp[i];
  }
  udpBuffer[len] = 0;

  parseControlData(udpBuffer);
}

void parseControlData(uint16_t *data) {
  char copy[255];

  // Convert uint16_t → char untuk parsing
  int i = 0;
  while (data[i] != 0 && i < 254) {
    copy[i] = (char)data[i];
    i++;
  }
  copy[i] = 0;

  // Tokenisasi
  char *token = strtok(copy, ",");
  while (token != NULL) {

    if (strncmp(token, "A:", 2) == 0)
      control.servoAngle = atof(token + 2);

    else if (strncmp(token, "M:", 2) == 0)
      control.motorPWM = constrain(atof(token + 2), 0, 999);

    token = strtok(NULL, ",");
  }

  Serial.printf("[Laptop] A=%.1f | M=%d\n", control.servoAngle, control.motorPWM);
}

// ====================================================
//     SEND TO STM32 VIA UART
// ====================================================
void sendToSTM32() {
  Serial1.printf("<A:%.1f,M:%d>", control.servoAngle, control.motorPWM);
}

// ====================================================
//     RECEIVE FROM STM32
// ====================================================
void receiveFromSTM32() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    uartBuffer[uartIndex++] = c;
    if (uartIndex >= 254) uartIndex = 0;

    if (c == '>') {
      uartBuffer[uartIndex] = 0;
      parseSTM32Data(uartBuffer);
      uartIndex = 0;
    }
  }
}

// ====================================================
//     PARSING STM32 → ESP32
// ====================================================
void parseSTM32Data(uint16_t *data) {
  char copy[255];
  int i = 0;

  // Convert uint16 → char
  while (data[i] != 0 && i < 254) {
    copy[i] = (char)data[i];
    i++;
  }
  copy[i] = 0;

  // Contoh format dari STM32:
  // <D:12.34,M:123,S:90.0>

  char *start = strchr(copy, '<');
  char *end   = strchr(copy, '>');
  if (!start || !end) return;

  char cpy[255];
  strncpy(cpy, start + 1, end - start - 1);
  cpy[end - start - 1] = 0;

  char *token = strtok(cpy, ",");
  while (token != NULL) {

    if (strncmp(token, "D:", 2) == 0)
      sensorData.distance = atof(token + 2);

    else if (strncmp(token, "M:", 2) == 0)
      sensorData.motorSpeed = atoi(token + 2);

    else if (strncmp(token, "S:", 2) == 0)
      sensorData.servoAngle = atof(token + 2);

    token = strtok(NULL, ",");
  }

  sensorData.obstacle = (sensorData.distance < 20);

  // Kirim ke laptop
  if (Udp.remotePort() != 0) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.printf("DIST=%.2f,OBS=%d,M=%d,S=%.1f",
       sensorData.distance, sensorData.obstacle,
       sensorData.motorSpeed, sensorData.servoAngle);
    Udp.endPacket();
  }
}