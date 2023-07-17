#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <common/mavlink.h>

#include "conf.h"
#include "creds.h"
#include "func.h"

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);

  Serial.begin(MAVLINK_BAUD);

  init_wifi();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousHeartbeatMillis >= heartbeat_interval) {
    previousHeartbeatMillis = currentMillis;

    send_heartbeat();
    send_systemstatus();
    send_radiostatus();
    send_position();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("\nReconnecting to ");
      Serial.println(WIFI_SSID);

      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);

      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  if (currentMillis - previousTimeoutMillis >= timeout_interval) {
    previousTimeoutMillis = millis();

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }

  while (Serial.available() > 0) {
    uint8_t serial_byte = Serial.read();
    parse_mavlink(serial_byte);
  }

  int packetSize = mavlink_udp.parsePacket();
  if (packetSize) {
    uint8_t packetBuffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_udp.read(packetBuffer, MAVLINK_MAX_PACKET_LEN);

    for (int16_t i = 0; i < packetSize; i++) {
      parse_mavlink(packetBuffer[i]);
    }
  }
}
