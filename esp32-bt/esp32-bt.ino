#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const char* remoteDeviceName = "DESKTOPFAHRIL";  // Nama laptop kamu
const int ledPin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  if (!SerialBT.begin("ESP32_Client", true)) { // true = mode client
    Serial.println("Gagal inisialisasi Bluetooth!");
    while (1);
  }

  Serial.println("Mencari device...");
  if (SerialBT.connect(remoteDeviceName)) {
    Serial.print("Terhubung ke ");
    Serial.println(remoteDeviceName);
  } else {
    Serial.println("Gagal connect, coba lagi...");
  }
}

void loop() {
  if (SerialBT.connected() && SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("Diterima: ");
    Serial.println(c);

    if (c >= '0' && c <= '9') {
      int jumlah = c - '0';
      for (int i = 0; i < jumlah; i++) {
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
        delay(300);
      }
    }
  }
}
