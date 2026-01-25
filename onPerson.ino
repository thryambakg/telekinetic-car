#include <WiFi.h>
#include <esp_now.h>

int8_t angle = 0;
uint8_t slaveMAC[] = {0x88, 0x57, 0x21, 0x78, 0xA7, 0xEC};

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, slaveMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int deg = line.toInt();
    deg = constrain(deg, -90, 90);

    angle = (int8_t)deg;
    
    esp_err_t result = esp_now_send(slaveMAC, (uint8_t *)&angle, sizeof(angle));
    
    if (result != ESP_OK) {
      Serial.println("Send error");
    }
  }
}

void onSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Callback when data is sent
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Sent: ");
    Serial.println(angle);
  } 
  else {
    Serial.println("Send failed!");
  }
}