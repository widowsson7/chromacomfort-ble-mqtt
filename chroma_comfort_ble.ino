/*
  ChromaComfort BLE -> MQTT bridge for Home Assistant.

  ESP32 firmware that bridges a Broan ChromaComfort smart bathroom fan (BLE) to
  Home Assistant over MQTT (via ArduinoHA). It both receives status updates from
  the fan and sends it commands, so Home Assistant stays in sync even when the
  fan is operated from its physical wall switches.

  Based on the original classic-Bluetooth sketch by Taylor Finnell
  (https://gist.github.com/taylorfinnell/5349b8085d57836a45be7637055e0692),
  rewritten for BLE using NimBLE, with the BLE protocol re-derived from sniffer
  captures. See README.md for the full reverse-engineered protocol. MIT.

  SETUP: copy config.example.h to config.h and fill in your WiFi / MQTT broker /
  fan MAC. config.h is gitignored so your credentials never get committed.
*/

#include <Arduino.h>
#include <ArduinoHA.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <cppQueue.h>

// All user-specific config (WiFi, MQTT broker, fan MAC) lives in config.h.
// Copy config.example.h to config.h and edit it. config.h is gitignored.
#include "config.h"

// Fan BLE MAC address (defined as FAN_MAC in config.h).
uint8_t address[6] = FAN_MAC;

#define DEVICE_PIN "1234"

// BLE UUIDs (confirmed from live on-device GATT discovery)
//
// The fan has two custom services. The DECOY service 00001016-... (chars
// 1013/1018/1014/1011) is ignored by the fan. The real CONTROL service is
// a08f7710-... and contains two distinct characteristics we use:
//   - status (notify)  -> b34ae89e-...  props 0x10 (Notify),            handle 0x001a
//   - command (write)  -> bb8a27e0-...  props 0x04 (Write No Response), handle 0x001d
// (The old Bluedroid BLEDevice library is still unusable here because NimBLE's
// vector-based discovery and per-property selection are what make this clean.)
#define CONTROL_SERVICE_UUID "a08f7710-c37c-11e3-99cc-0228ac012a70"
#define WRITE_CHARACTERISTIC_UUID "bb8a27e0-c37c-11e3-b953-0228ac012a70"
#define NOTIFY_CHARACTERISTIC_UUID "b34ae89e-c37c-11e3-940e-0228ac012a70"

#define CODE_CMD_START 58
#define CODE_CMD_LENGTH 17
#define CODE_VERSION 1
#define CODE_CTRL_CMD_1 0
#define CODE_CTRL_CMD_2 64
#define CODE_TURN_FAN_ON 1
#define CODE_TURN_FAN_OFF 2
#define CODE_TURN_LIGHT_ON 3
#define CODE_TURN_LIGHT_OFF 4
#define CODE_TURN_ON_RGB 5
#define CODE_TURN_OFF_RGB 6
#define CODE_SAVE_FAVORITE_COLOR1 13
#define CODE_ACTIVATE_FAVORITE_COLOR1 11
#define CODE_DEACTIVATE_FAVORITE_COLOR1 12
#define CODE_COUNTDOWN_ON 17
#define CODE_COUNTDOWN_OFF 18
#define CODE_SAVE_CUSTOM_PATTERN 42
#define CODE_ACTIVATE_CUSTOM_PATTERN 32
#define CODE_DEACTIVATE_CUSTOM_PATTERN 33

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
int lastUpdateAt = 0;
unsigned long lastBleAttemptAt = 0;  // for non-blocking BLE (re)connect timer
int32_t acks = 0;
int32_t txs = 0;
int32_t rxs = 0;
int32_t heartbeats = 0;
int32_t panics = 0;

HASwitch fan("fan");
HASwitch wallRgb("wallrgb");
// "light" is the RGB favorite-color light (status bit 3). "whitelight" is the
// plain bright-white light (status bit 6). On the fan these two modes plus the
// Wall RGB color-cycle (bit 5) are mutually exclusive — activating one clears
// the others — so in HA they behave like radio buttons, which is accurate.
HALight light("light", HALight::BrightnessFeature | HALight::RGBFeature);
HALight whiteLight("whitelight", HALight::BrightnessFeature);
HASensorNumber uptimeSensor("uptime");
HASensorNumber errorSensor("errors");
HASensorNumber txSensor("tx");
HASensorNumber rxSensor("rx");
HASensorNumber acksSensor("acks");

// BLE client and characteristic pointers
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pWriteChar = nullptr;
NimBLERemoteCharacteristic* pNotifyChar = nullptr;
bool bleConnected = false;

typedef struct Packet {
  byte header;
  byte len;
  uint8_t data[CODE_CMD_LENGTH];
};

typedef struct TxCmd {
  byte version = CODE_VERSION;
  byte ctrl_cmd_1 = CODE_CTRL_CMD_1;
  byte ctrl_cmd_2 = CODE_CTRL_CMD_2;
  byte type = 0;
  byte r = 0;
  byte g = 0;
  byte b = 0;
  byte dimmer = 0;
  byte speed = 0;
  byte sweep_color_value_1 = 1;
  byte sweep_color_value_2 = 24;
  byte duration = 0;
  byte timer_1_value = 0;
  byte timer_2_value = 0;
  byte timer_3_value = 0;
  byte timer_4_value = 0;
  byte data_end = 0;
};

typedef struct StatusRxCmd {
  byte version = 5;
  byte control1;
  byte control2;
  byte status_mask;
  byte unk;
  byte brightness;
};

typedef struct AckRxCmd {
  byte version = 5;
  byte control1;
  byte control2;
  byte end;
};

int applyGammaCorrection(int r, double t) {
  return (int)(255 * pow((double)r / 255, t));
}

#define IMPLEMENTATION FIFO

#define NB_ITEMS 25
Packet t_dat[NB_ITEMS];
Packet r_dat[NB_ITEMS];

cppQueue tx(
    sizeof(Packet), NB_ITEMS, IMPLEMENTATION, false, t_dat,
    sizeof(t_dat));
cppQueue rx(
    sizeof(Packet), NB_ITEMS, IMPLEMENTATION, false, r_dat,
    sizeof(r_dat));

Packet createPacket(uint8_t header, const uint8_t* data, size_t dataSize) {
  Packet packet;
  packet.header = header;
  packet.len = (uint8_t)dataSize;
  memcpy(packet.data, data, dataSize);
  return packet;
}

void turnFanOn() {
  TxCmd cmd;
  cmd.type = CODE_TURN_FAN_ON;
  Packet packet = createPacket(58, (const uint8_t*)&cmd, 17);
  tx.push(&packet);
}

void activateFavColor(byte brightness) {
  TxCmd cmd1;
  cmd1.type = CODE_ACTIVATE_FAVORITE_COLOR1;
  cmd1.dimmer = brightness;
  Packet packet = createPacket(58, (const uint8_t*)&cmd1, 17);
  tx.push(&packet);
}

void setRGB(byte r, byte g, byte b) {
  TxCmd cmd0;
  cmd0.type = CODE_SAVE_FAVORITE_COLOR1;
  cmd0.r = r;
  cmd0.g = g;
  cmd0.b = b;
  cmd0.speed = 30;
  Packet packet = createPacket(58, (const uint8_t*)&cmd0, 17);
  tx.push(&packet);
}

void deactivateFavColor() {
  TxCmd cmd1;
  cmd1.type = CODE_DEACTIVATE_FAVORITE_COLOR1;
  Packet packet = createPacket(58, (const uint8_t*)&cmd1, 17);
  tx.push(&packet);
}

void turnOnWallRGB() {
  TxCmd cmd1;
  cmd1.type = CODE_TURN_ON_RGB;
  Packet packet = createPacket(58, (const uint8_t*)&cmd1, 17);
  tx.push(&packet);
}

void turnOffWallRGB() {
  TxCmd cmd1;
  cmd1.type = CODE_TURN_OFF_RGB;
  Packet packet = createPacket(58, (const uint8_t*)&cmd1, 17);
  tx.push(&packet);
}

void turnFanOff() {
  TxCmd cmd;
  cmd.type = CODE_TURN_FAN_OFF;
  Packet packet = createPacket(58, (const uint8_t*)&cmd, 17);
  tx.push(&packet);
}

void turnWhiteLightOn(byte brightness) {
  TxCmd cmd;
  cmd.type = CODE_TURN_LIGHT_ON;  // 3 - bright white light, dimmer = brightness
  cmd.dimmer = brightness;
  Packet packet = createPacket(58, (const uint8_t*)&cmd, 17);
  tx.push(&packet);
}

void turnWhiteLightOff() {
  TxCmd cmd;
  cmd.type = CODE_TURN_LIGHT_OFF;  // 4
  Packet packet = createPacket(58, (const uint8_t*)&cmd, 17);
  tx.push(&packet);
}

int fromHABrightness(uint8_t b) {
  float x = (float)b / 255.0f;
  x *= 100.0f;
  return (int)x;
}

int toHABrightness(uint8_t b) {
  float x = (float)b / 100.0f;
  x *= 255.0f;
  return (int)x;
}

void onSwitchCommandFan(bool state, HASwitch* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    turnFanOn();
  } else {
    turnFanOff();
  }
  sender->setState(state);
}

void onSwitchCommandWallRgb(bool state, HASwitch* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    turnOnWallRGB();
  } else {
    turnOffWallRGB();
  }

  sender->setState(state);
}

void onStateCommand(bool state, HALight* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    activateFavColor(fromHABrightness(sender->getCurrentBrightness()));
  } else {
    deactivateFavColor();
  }
  sender->setState(state);
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
  if ((int)brightness == (int)sender->getCurrentBrightness()) {
    return;
  }
  activateFavColor(fromHABrightness(brightness));
  sender->setBrightness(brightness);
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
  // The official app sends raw 0-255 RGB to the fan with NO gamma correction
  // (confirmed by BLE capture: e.g. orange -> 0d ff a5 04). Send values as-is.
  setRGB(color.red, color.green, color.blue);

  sender->setRGBColor(color);
}

void onWhiteStateCommand(bool state, HALight* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    turnWhiteLightOn(fromHABrightness(sender->getCurrentBrightness()));
  } else {
    turnWhiteLightOff();
  }
  sender->setState(state);
}

void onWhiteBrightnessCommand(uint8_t brightness, HALight* sender) {
  if ((int)brightness == (int)sender->getCurrentBrightness()) {
    return;
  }
  turnWhiteLightOn(fromHABrightness(brightness));
  sender->setBrightness(brightness);
}

void printPacket(Packet packet) {
  Serial.printf("Packet (len=%d, header=%d): ", packet.len, packet.header);
  Serial.printf("0x%02X ", packet.header);
  Serial.printf("0x%02X ", packet.len);

  for (int i = 0; i < packet.len; i++) {
    Serial.printf("0x%02X ", packet.data[i]);
  }
  Serial.print("\r\n");
}

void onBLENotify(NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
                 uint8_t* pData, size_t length, bool isNotify) {
  if (length < 2) {
    return;
  }

  if (pData[0] != 58) {
    Serial.printf("Got invalid packet: %d, %d\r\n", length, pData[1]);
    return;
  }

  Packet packet = createPacket(pData[0], pData + 2, pData[1]);
  printPacket(packet);
  rx.push(&packet);
}

class BLEClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    Serial.println("BLE Connected");
    bleConnected = true;
  }

  void onDisconnect(NimBLEClient* pclient, int reason) {
    Serial.printf("BLE Disconnected (reason %d)\r\n", reason);
    bleConnected = false;
  }
};

// Clean up the current client and return false. Every failure path must go
// through here: NimBLE caps concurrent clients (~3), so leaking a client on
// each retry exhausts the pool and the next createClient() returns nullptr,
// which then crashes (LoadProhibited) on the first method call.
bool bleFail(const char* msg) {
  Serial.println(msg);
  if (pClient != nullptr) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }
  pWriteChar = nullptr;
  pNotifyChar = nullptr;
  return false;
}

bool connectToBLE() {
  Serial.print("Forming a connection to ");

  NimBLEAddress bleAddress(address, BLE_ADDR_PUBLIC);
  Serial.println(bleAddress.toString().c_str());

  pWriteChar = nullptr;
  pNotifyChar = nullptr;

  // Delete any previous client (e.g. left over from a runtime disconnect) so we
  // never leak clients and exhaust NimBLE's small client pool across reconnects.
  if (pClient != nullptr) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }

  pClient = NimBLEDevice::createClient();
  if (pClient == nullptr) {
    return bleFail("createClient() returned null (client pool exhausted)");
  }
  // One persistent callback instance (deleteCallbacks=false), so reconnecting
  // doesn't leak a callback object each time.
  static BLEClientCallback bleCallbacks;
  pClient->setClientCallbacks(&bleCallbacks, false);
  // Bound a failed attempt so an unreachable fan can't stall the main loop (and
  // therefore MQTT) for NimBLE's default ~30s.
  pClient->setConnectTimeout(10000);  // milliseconds

  if (!pClient->connect(bleAddress)) {
    return bleFail("Failed to connect to BLE device");
  }

  Serial.println("Connected to BLE Device");

  NimBLEUUID writeUuid(WRITE_CHARACTERISTIC_UUID);
  NimBLEUUID notifyUuid(NOTIFY_CHARACTERISTIC_UUID);

  // The command and status characteristics live in the a08f7710-... service and
  // have distinct UUIDs, so just match each by UUID. Scan every service so we
  // don't depend on discovery order.
  const std::vector<NimBLERemoteService*>& services = pClient->getServices(true);
  for (NimBLERemoteService* svc : services) {
    const std::vector<NimBLERemoteCharacteristic*>& chars =
        svc->getCharacteristics(true);
    for (NimBLERemoteCharacteristic* c : chars) {
      if (c->getUUID().equals(writeUuid)) {
        pWriteChar = c;
      } else if (c->getUUID().equals(notifyUuid)) {
        pNotifyChar = c;
      }
    }
  }

  if (pWriteChar == nullptr) {
    return bleFail("Failed to find the command characteristic");
  }
  Serial.printf("Found command characteristic (handle 0x%04x)\r\n",
                pWriteChar->getHandle());

  if (pNotifyChar == nullptr) {
    return bleFail("Failed to find the status (notify) characteristic");
  }
  Serial.printf("Found status characteristic (handle 0x%04x)\r\n",
                pNotifyChar->getHandle());

  // Subscribe for notifications. NimBLE writes the CCCD (0x2902) for us, which
  // is also what makes the fan start processing commands. true = notifications.
  if (!pNotifyChar->subscribe(true, onBLENotify)) {
    return bleFail("Failed to subscribe to status notifications");
  }
  Serial.println("Subscribed to status notifications");

  bleConnected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  byte mac[6];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected to the network");

  device.setName("ChromaComfort BT to MQTT Bridge");
  device.setSoftwareVersion("1.0.0");

  fan.onCommand(onSwitchCommandFan);
  fan.setName("Fan");

  wallRgb.onCommand(onSwitchCommandWallRgb);
  wallRgb.setName("Wall RGB");

  light.onStateCommand(onStateCommand);
  light.onBrightnessCommand(onBrightnessCommand);
  light.onRGBColorCommand(onRGBColorCommand);
  light.setName("Color Light");

  whiteLight.onStateCommand(onWhiteStateCommand);
  whiteLight.onBrightnessCommand(onWhiteBrightnessCommand);
  whiteLight.setName("White Light");

  uptimeSensor.setIcon("mdi:home");
  uptimeSensor.setName("Uptime");
  uptimeSensor.setUnitOfMeasurement("s");
  errorSensor.setIcon("mdi:alert-circle");
  errorSensor.setName("Errors");
  txSensor.setIcon("mdi:transfer-up");
  txSensor.setName("TX");
  rxSensor.setIcon("mdi:transfer-down");
  rxSensor.setName("RX");
  acksSensor.setIcon("mdi:call-received");
  acksSensor.setName("Acks");

  mqtt.begin(BROKER_ADDR, MQTT_USER, MQTT_PASS);
  Serial.println("Connected to mqtt");

  NimBLEDevice::init("");

  // Attempt the first BLE connection, but DON'T block here if it fails. loop()
  // retries on a timer, so MQTT / Home Assistant come up immediately even when
  // the fan is unreachable at boot (the bridge just shows its entities and
  // (re)connects to the fan in the background).
  Serial.println("Attempting initial BLE connection (non-blocking)...");
  connectToBLE();
}

int heartbeats_since_ack = 0;
bool firstRun = true;

void loop() {
  mqtt.loop();

  if (!firstRun) {
    acksSensor.setValue((int32_t)0);
    rxSensor.setValue((int32_t)0);
    txSensor.setValue((int32_t)0);
    errorSensor.setValue((int32_t)0);
    uptimeSensor.setValue((int32_t)0);
    firstRun = false;
  }

  if (!bleConnected) {
    // Non-blocking reconnect: retry on a timer instead of delay()-looping, so
    // mqtt.loop() (called above every iteration) keeps Home Assistant alive
    // while the fan is unreachable. Skip the TX/RX handling until BLE is back.
    if (millis() - lastBleAttemptAt > 5000) {
      lastBleAttemptAt = millis();
      Serial.println("BLE not connected, attempting reconnect...");
      connectToBLE();
    }
    return;
  }

  if ((millis() - lastUpdateAt) > 2000) {
    unsigned long uptimeValue = millis() / 1000;
    uptimeSensor.setValue((int32_t)uptimeValue);
    heartbeats += 1;
    if (tx.getCount() > 0) {
      heartbeats_since_ack += 1;
    }
    lastUpdateAt = millis();
    rxSensor.setValue(rxs);
  }

  if (tx.getCount() > 0) {
    Serial.printf("It's been %d heartbeats since last ack",
                  heartbeats_since_ack);

    Packet packet;
    tx.peek(&packet);

    Serial.println("About to send packet: ");
    printPacket(packet);

    if (pWriteChar != nullptr) {
      pWriteChar->writeValue((uint8_t*)&packet, sizeof(packet), false);
      delay(37);
      pWriteChar->writeValue((uint8_t*)&packet, sizeof(packet), false);
      delay(37);
      pWriteChar->writeValue((uint8_t*)&packet, sizeof(packet), false);

      tx.pop(&packet);
      txs += 1;
      txSensor.setValue(txs);
    } else {
      Serial.println("Write characteristic not available");
    }
  }

  if (rx.getCount() > 0) {
    rxs += 1;

    Packet packet;
    rx.pop(&packet);

    if (packet.len == 4 && tx.getCount() > 0) {
      AckRxCmd* ack = (AckRxCmd*)(packet.data);

      if (ack->control1 == 160 && ack->control2 == 64) {
        acks += 1;
        acksSensor.setValue(acks);
        tx.pop(&packet);
        heartbeats_since_ack = 0;
      }
    }

    if (packet.len == 17 && tx.getCount() == 0) {
      StatusRxCmd* status = (StatusRxCmd*)(packet.data);

      if (status->control1 == 160 && status->control2 == 65) {
        Serial.println("Status update...");

        byte s = status->status_mask;
        bool isFanOn = ((s >> 7) & 1) == 1;
        bool isLightOn = ((s >> 6) & 1) == 1;
        bool rgbButton = ((s >> 5) & 1) == 1;
        bool rgbSweep = ((s >> 4) & 1) == 1;
        bool isFavoriteColor1Active = ((s >> 3) & 1) == 1;
        bool isFavoriteColor2Active = ((s >> 2) & 1) == 1;
        bool userPattern = ((s >> 1) & 1) == 1;
        bool reservedBit = ((s >> 0) & 1) == 1;
        int brightness = status->brightness;

        Serial.printf(
            "Light: %d, Fan: %d, isFavoriteColor1Active: %d, wallRGBButton: "
            "%d, Brightness: %d\r\n",
            isLightOn, isFanOn, isFavoriteColor1Active, rgbButton, brightness);

        // The three light modes are mutually exclusive on the fan, so map each
        // status bit to its own entity. The single brightness byte belongs to
        // whichever mode is currently active.
        fan.setState(isFanOn);
        whiteLight.setState(isLightOn);              // bit 6 - bright white
        light.setState(isFavoriteColor1Active);      // bit 3 - RGB color
        wallRgb.setState(rgbButton);                 // bit 5 - factory cycle
        if (isLightOn) {
          whiteLight.setBrightness(toHABrightness(brightness));
        }
        if (isFavoriteColor1Active) {
          light.setBrightness(toHABrightness(brightness));
        }
      }
    }
  }

  if (heartbeats_since_ack > 2) {
    Serial.println("Flushing the queue from panic");
    tx.flush();
    heartbeats_since_ack = 0;
    panics += 1;
    errorSensor.setValue(panics);
  }
}