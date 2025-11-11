#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Ijul";       
const char* password = "12345678"; 
const int udpPort = 50002;                   

WiFiUDP Udp;
char incomingPacket[255]; // Buffer untuk menyimpan data yang masuk

void setup() {
  Serial.begin(115200);
  
  // 1. Sambungkan ke Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan ke Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nTersambung!");
  Serial.print("IP Lokal ESP32: ");
  Serial.println(WiFi.localIP());

  // 2. Mulai Mendengarkan UDP
  Udp.begin(udpPort);
  Serial.print("UDP Mendengarkan di Port: ");
  Serial.println(udpPort);
}

void loop() {
  // 3. Cek Paket UDP yang Masuk
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Membaca paket ke dalam buffer
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Menambahkan null terminator
    }
    
    // Tampilkan data yang diterima
    Serial.println("\n--- Data Diterima dari Laptop ---");
    Serial.print("Isi Pesan: ");
    Serial.println(incomingPacket);
  }
}