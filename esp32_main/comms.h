/*#ifndef COMMS_H
#define COMMS_H

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x64, 0xB7, 0x08, 0x9C, 0x65, 0x90};

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

#endif*/