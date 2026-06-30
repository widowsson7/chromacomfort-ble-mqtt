#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
//  Configuration template. Copy this file to config.h and fill in your values:
//
//      cp config.example.h config.h
//
//  config.h is gitignored so your credentials never get committed.
// =============================================================================

// Fan BLE MAC address, as { b0, b1, b2, b3, b4, b5 }.
// Find it with a BLE scanner app (e.g. nRF Connect) - it's the ChromaComfort.
#define FAN_MAC { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

// WiFi
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// MQTT broker (your Home Assistant / Mosquitto broker)
#define BROKER_ADDR IPAddress(192, 168, 1, 100)
#define MQTT_USER "your-mqtt-username"
#define MQTT_PASS "your-mqtt-password"

#endif  // CONFIG_H
