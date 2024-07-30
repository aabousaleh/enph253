/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of the other ESP32
uint8_t broadcastAddress[] = {0x64, 0xB7, 0x08, 0x9C, 0x65, 0x90}; //64:b7:08:9c:65:90 left robot

long timeStart;
long timeEnd;

char dataToBeSent[20] = "RK Robot 1 Message"; //change for each esp

char dataReceived[20] = "default 1 message";

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Serial.print("Incoming Data as is: ");
  // Serial.println(String(*incomingData));
  memcpy(&dataReceived, incomingData, 20);
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println(dataReceived);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  delay(3000);
}
 
void loop() {

  // Send message via ESP-NOW
  timeStart = micros();
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToBeSent, sizeof(dataToBeSent));
  timeEnd = micros();
   
  if (result == ESP_OK) {
    Serial.println("Sent with success, possible cap");
    Serial.print("Data received: ");
    Serial.println(dataReceived);
    Serial.println(timeEnd - timeStart);
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000);
}
