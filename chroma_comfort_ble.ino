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
#include <ArduinoOTA.h>    // wireless firmware updates over WiFi
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <cppQueue.h>
#include "esp_task_wdt.h"  // hardware Task Watchdog (ESP32 core 3.x API)
#include "esp_system.h"    // esp_reset_reason() for unlogged-reset forensics
#include <Preferences.h>   // persist crash diagnostics across a self-heal restart

// All user-specific config (WiFi, MQTT broker, fan MAC) lives in config.h.
// Copy config.example.h to config.h and edit it. config.h is gitignored.
#include "config.h"

// Serial diagnostics. Set CC_DEBUG to 0 (or add -DCC_DEBUG=0 to build_opt.h)
// to compile out every log string and printf call, freeing flash. Default is
// on, matching the historical behavior.
#ifndef CC_DEBUG
#define CC_DEBUG 1
#endif
#if CC_DEBUG
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#define DBG_PRINTF(...)
#endif

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
// 3rd arg = max device types (entities). We declare 12 below (fan, wallRgb,
// light, whiteLight, 7 sensors, reboot); ArduinoHA's default cap can be lower,
// which would silently drop the overflow and break their discovery. Size it
// generously so every entity registers.
HAMqtt mqtt(client, device, 15);
Preferences crashPrefs;
String lastRestartInfo = "none";
unsigned long lastUpdateAt = 0;
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
HASensorNumber rssiSensor("rssi");
HASensor lastRestartSensor("lastrestart");
HAButton rebootButton("reboot");  // remote reboot from the HA dashboard

// Self-heal: reboot the ESP if a subsystem stays down past its threshold. This
// recovers from wedged states (e.g. a hung BLE host stack) that in-loop
// reconnect can't clear. Software ESP.restart() is used (not esp_task_wdt),
// which is portable across ESP32 core versions.
const unsigned long WIFI_DOWN_REBOOT_MS = 60000;   // 60s
const unsigned long MQTT_DOWN_REBOOT_MS = 120000;  // 120s
const unsigned long BLE_DOWN_REBOOT_MS = 90000;    // 90s (after ~18 retries)
// Hardware Task Watchdog timeout. Must comfortably exceed the longest legit
// loop() iteration (a BLE connect attempt can block ~10-30s), so set it well
// above that - this only fires on a TRUE freeze where loop() stops entirely.
const uint32_t WDT_TIMEOUT_MS = 60000;             // 60s
unsigned long lastWifiOkAt = 0;
unsigned long lastMqttOkAt = 0;
unsigned long lastBleOkAt = 0;

// Ultimate catch-all: a scheduled clean reboot bounds ANY undetectable stuck
// state to at most this interval, even ones the watchdog/self-heal can't see.
// (HA also now triggers a clean reboot every 6h on a wall-clock schedule via
// the "Bathroom Fan - Bridge Scheduled Reboot" automation; this is a backstop
// in case that automation or MQTT itself is what's down.)
const unsigned long PROACTIVE_REBOOT_MS = 86400000;  // 24h

// BLE client and characteristic pointers
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pWriteChar = nullptr;
NimBLERemoteCharacteristic* pNotifyChar = nullptr;
bool bleConnected = false;

struct Packet {
  byte header;
  byte len;
  uint8_t data[CODE_CMD_LENGTH];
};

struct TxCmd {
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

struct StatusRxCmd {
  byte version = 5;
  byte control1;
  byte control2;
  byte status_mask;
  byte unk;
  byte brightness;
};

struct AckRxCmd {
  byte version = 5;
  byte control1;
  byte control2;
  byte end;
};

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

// The rx queue is written from the NimBLE host task (onBLENotify runs in its
// GAP event handler) and drained from the Arduino loop task, and cppQueue has
// no internal locking - unguarded, the two tasks race on the queue's count/
// index fields and can corrupt it (stuck-full queue, phantom packets). Guard
// every rx access with this spinlock; the critical sections are only a 19-byte
// memcpy, so the lock hold time is negligible.
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

Packet createPacket(uint8_t header, const uint8_t* data, size_t dataSize) {
  Packet packet;
  packet.header = header;
  packet.len = (uint8_t)dataSize;
  memcpy(packet.data, data, dataSize);
  return packet;
}

// Build one 19-byte command frame and queue it for the BLE writer in loop().
// All twelve fan/light commands share this one frame layout; only the type
// byte and (for some commands) dimmer/RGB/speed differ. Protocol rules from
// README.md are enforced here in one place: version stays CODE_VERSION (0x01),
// control bytes stay 00/40, and speed stays 0 for everything except the
// color-save command (0x0D), which is the only caller that passes speed=30.
void queueCommand(byte type, byte dimmer = 0, byte r = 0, byte g = 0,
                  byte b = 0, byte speed = 0) {
  TxCmd cmd;
  cmd.type = type;
  cmd.dimmer = dimmer;
  cmd.r = r;
  cmd.g = g;
  cmd.b = b;
  cmd.speed = speed;
  Packet packet = createPacket(CODE_CMD_START, (const uint8_t*)&cmd,
                               CODE_CMD_LENGTH);
  // The queue drops the newest item when full (overwrite=false). That only
  // happens when BLE has been down long enough for 25 commands to pile up;
  // surface it instead of losing the command silently.
  if (!tx.push(&packet)) {
    DBG_PRINTLN("TX queue full, command dropped");
    panics += 1;
    errorSensor.setValue(panics);
  }
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

  queueCommand(state ? CODE_TURN_FAN_ON : CODE_TURN_FAN_OFF);
  sender->setState(state);
}

void onSwitchCommandWallRgb(bool state, HASwitch* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  queueCommand(state ? CODE_TURN_ON_RGB : CODE_TURN_OFF_RGB);
  sender->setState(state);
}

void onStateCommand(bool state, HALight* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    queueCommand(CODE_ACTIVATE_FAVORITE_COLOR1,
                 fromHABrightness(sender->getCurrentBrightness()));
  } else {
    queueCommand(CODE_DEACTIVATE_FAVORITE_COLOR1);
  }
  sender->setState(state);
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
  if ((int)brightness == (int)sender->getCurrentBrightness()) {
    return;
  }
  queueCommand(CODE_ACTIVATE_FAVORITE_COLOR1, fromHABrightness(brightness));
  sender->setBrightness(brightness);
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
  // The official app sends raw 0-255 RGB to the fan with NO gamma correction
  // (confirmed by BLE capture: e.g. orange -> 0d ff a5 04). Send values as-is.
  // Color-save (0x0D) is the ONE command that uses a non-zero speed byte (30).
  queueCommand(CODE_SAVE_FAVORITE_COLOR1, 0, color.red, color.green,
               color.blue, 30);

  sender->setRGBColor(color);
}

void onWhiteStateCommand(bool state, HALight* sender) {
  if (state == sender->getCurrentState()) {
    return;
  }

  if (state) {
    queueCommand(CODE_TURN_LIGHT_ON,
                 fromHABrightness(sender->getCurrentBrightness()));
  } else {
    queueCommand(CODE_TURN_LIGHT_OFF);
  }
  sender->setState(state);
}

void onWhiteBrightnessCommand(uint8_t brightness, HALight* sender) {
  if ((int)brightness == (int)sender->getCurrentBrightness()) {
    return;
  }
  queueCommand(CODE_TURN_LIGHT_ON, fromHABrightness(brightness));
  sender->setBrightness(brightness);
}

void printPacket(Packet packet) {
#if CC_DEBUG
  Serial.printf("Packet (len=%d, header=%d): ", packet.len, packet.header);
  Serial.printf("0x%02X ", packet.header);
  Serial.printf("0x%02X ", packet.len);

  for (int i = 0; i < packet.len; i++) {
    Serial.printf("0x%02X ", packet.data[i]);
  }
  Serial.print("\r\n");
#endif
}

void onBLENotify(NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
                 uint8_t* pData, size_t length, bool isNotify) {
  if (length < 2) {
    return;
  }

  if (pData[0] != 58) {
    DBG_PRINTF("Got invalid packet: %d, %d\r\n", length, pData[1]);
    return;
  }

  // The wire length byte is attacker/corruption-controlled; clamp it before
  // createPacket() memcpys it into Packet.data[17] (a bad byte here would
  // otherwise smash the NimBLE host task stack) and never read past the
  // actual notification payload.
  uint8_t dataLen = pData[1];
  if (dataLen > CODE_CMD_LENGTH || (size_t)dataLen + 2 > length) {
    DBG_PRINTF("Got oversized packet: %d, %d\r\n", length, dataLen);
    return;
  }

  Packet packet = createPacket(pData[0], pData + 2, dataLen);
  portENTER_CRITICAL(&rxMux);
  rx.push(&packet);
  portEXIT_CRITICAL(&rxMux);
}

class BLEClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    DBG_PRINTLN("BLE Connected");
    bleConnected = true;
  }

  void onDisconnect(NimBLEClient* pclient, int reason) {
    DBG_PRINTF("BLE Disconnected (reason %d)\r\n", reason);
    bleConnected = false;
  }
};

// Clean up the current client and return false. Every failure path must go
// through here: NimBLE caps concurrent clients (~3), so leaking a client on
// each retry exhausts the pool and the next createClient() returns nullptr,
// which then crashes (LoadProhibited) on the first method call.
bool bleFail(const char* msg) {
  DBG_PRINTLN(msg);
  if (pClient != nullptr) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }
  pWriteChar = nullptr;
  pNotifyChar = nullptr;
  return false;
}

bool connectToBLE() {
  DBG_PRINT("Forming a connection to ");

  NimBLEAddress bleAddress(address, BLE_ADDR_PUBLIC);
  DBG_PRINTLN(bleAddress.toString().c_str());

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

  DBG_PRINTLN("Connected to BLE Device");

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
  DBG_PRINTF("Found command characteristic (handle 0x%04x)\r\n",
             pWriteChar->getHandle());

  if (pNotifyChar == nullptr) {
    return bleFail("Failed to find the status (notify) characteristic");
  }
  DBG_PRINTF("Found status characteristic (handle 0x%04x)\r\n",
             pNotifyChar->getHandle());

  // Subscribe for notifications. NimBLE writes the CCCD (0x2902) for us, which
  // is also what makes the fan start processing commands. true = notifications.
  if (!pNotifyChar->subscribe(true, onBLENotify)) {
    return bleFail("Failed to subscribe to status notifications");
  }
  DBG_PRINTLN("Subscribed to status notifications");

  bleConnected = true;
  return true;
}

// Persist why we're about to restart to NVS (flash) so it survives the reboot.
// Without this, a hang that only the watchdog/self-heal can see leaves no
// trace once the chip comes back up - you're left guessing from RSSI/heap
// snapshots that no longer reflect the state that actually caused the restart.
void saveCrashInfo(const char* reason) {
  crashPrefs.begin("chromacomfort", false);  // read-write
  crashPrefs.putString("reason", reason);
  crashPrefs.putULong("uptime_s", millis() / 1000);
  crashPrefs.putInt("rssi", WiFi.RSSI());
  crashPrefs.putUInt("heap", ESP.getFreeHeap());
  crashPrefs.putBool("wifi_ok", WiFi.status() == WL_CONNECTED);
  crashPrefs.putBool("mqtt_ok", mqtt.isConnected());
  crashPrefs.putBool("ble_ok", bleConnected);
  crashPrefs.end();
}

// Restart with a clean network teardown. A bare ESP.restart() abandons the
// MQTT socket without telling the broker, so the broker keeps the old session;
// when the device reconnects (same client id) publishing breaks - that's why a
// software reboot didn't recover a half-open state but a power cycle did. A
// clean MQTT disconnect + WiFi/radio off makes a software restart behave like a
// power cycle for the broker.
void cleanRestart(const char* reason) {
  DBG_PRINTF("Restarting: %s\r\n", reason);
  saveCrashInfo(reason);
  mqtt.disconnect();
  delay(150);
  WiFi.disconnect(true);
  delay(250);
  ESP.restart();
}

void onRebootButtonCommand(HAButton* sender) {
  cleanRestart("Home Assistant reboot button");
}

// ArduinoOTA callbacks. onProgress fires repeatedly while a firmware image is
// being received; ArduinoOTA.handle() blocks loop() for the whole transfer, so
// pet the hardware watchdog here or a slow WiFi upload can panic-reboot mid-
// flash and corrupt the OTA partition.
void onOTAStart() {
  DBG_PRINTF("OTA update starting (%s)...\r\n",
             (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  saveCrashInfo("OTA update in progress");
}

void onOTAEnd() {
  DBG_PRINTLN("OTA update complete, rebooting...");
}

void onOTAProgress(unsigned int progress, unsigned int total) {
  esp_task_wdt_reset();
#if CC_DEBUG
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 1000) {
    lastLog = millis();
    Serial.printf("OTA progress: %u%%\r\n", (progress * 100) / total);
  }
#endif
}

void onOTAError(ota_error_t error) {
  DBG_PRINTF("OTA error [%u]\r\n", (unsigned)error);
}

// Reboot if WiFi / MQTT / BLE stays down past its threshold. Each subsystem's
// "last OK" timestamp is refreshed while it's healthy; if any falls too far
// behind, restart. Covers hung states that in-loop reconnect can't recover.
void selfHeal() {
  unsigned long now = millis();
  if (WiFi.status() == WL_CONNECTED) lastWifiOkAt = now;
  if (mqtt.isConnected()) lastMqttOkAt = now;
  if (bleConnected) lastBleOkAt = now;

  // Ultimate catch-all: a scheduled clean reboot bounds ANY undetectable stuck
  // state (including a half-open MQTT publish failure self-heal can't see).
  if (now > PROACTIVE_REBOOT_MS) {
    cleanRestart("scheduled 24h proactive reboot");
  }

  const char* reason = nullptr;
  if (now - lastWifiOkAt > WIFI_DOWN_REBOOT_MS) reason = "WiFi down too long";
  else if (now - lastMqttOkAt > MQTT_DOWN_REBOOT_MS) reason = "MQTT down too long";
  else if (now - lastBleOkAt > BLE_DOWN_REBOOT_MS) reason = "BLE down too long";

  if (reason != nullptr) {
    cleanRestart(reason);
  }
}

void setup() {
  Serial.begin(115200);
  DBG_PRINTLN("Starting...");

  // Read back why the previous boot restarted, before anything else touches
  // NVS. The saved reason is ONE-SHOT: it's removed after being read, because
  // hard resets (task-WDT panic, brownout, power cycle) never call
  // saveCrashInfo, and without the removal they'd replay a stale reason from
  // an older soft restart - actively misleading during exactly the kind of
  // debugging this sensor exists for. When there's no saved reason, report the
  // silicon's own reset cause instead (esp_reset_reason: 1=poweron 3=sw
  // 4=panic 5=int-wdt 6=task-wdt 9=brownout).
  crashPrefs.begin("chromacomfort", false);  // read-write (to clear the reason)
  if (crashPrefs.isKey("reason")) {
    lastRestartInfo = crashPrefs.getString("reason", "unknown") + " (uptime=" +
        String(crashPrefs.getULong("uptime_s", 0)) + "s rssi=" +
        String(crashPrefs.getInt("rssi", 0)) + "dBm heap=" +
        String(crashPrefs.getUInt("heap", 0)) + " wifi=" +
        String(crashPrefs.getBool("wifi_ok", false)) + " mqtt=" +
        String(crashPrefs.getBool("mqtt_ok", false)) + " ble=" +
        String(crashPrefs.getBool("ble_ok", false)) + ")";
    crashPrefs.remove("reason");
    DBG_PRINTF("[boot] previous restart: %s\r\n", lastRestartInfo.c_str());
  } else {
    lastRestartInfo =
        "unlogged reset (esp_reset_reason=" + String((int)esp_reset_reason()) + ")";
  }
  crashPrefs.end();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    DBG_PRINT(".");
    delay(500);
    if (millis() - wifiStart > WIFI_DOWN_REBOOT_MS) {
      DBG_PRINTLN("\nWiFi connect timed out; restarting...");
      saveCrashInfo("WiFi connect timeout at boot");
      ESP.restart();
    }
  }
  DBG_PRINTLN();
  DBG_PRINTLN("Connected to the network");

  // Wireless firmware updates. Password-protected since this opens a network
  // port that accepts firmware pushes. ArduinoOTA.handle() in loop() blocks for
  // the whole transfer, so pet the hardware watchdog from onOTAProgress.
  ArduinoOTA.setHostname("chromacomfort-bridge");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart(onOTAStart);
  ArduinoOTA.onEnd(onOTAEnd);
  ArduinoOTA.onProgress(onOTAProgress);
  ArduinoOTA.onError(onOTAError);
  ArduinoOTA.begin();
  DBG_PRINTLN("OTA ready");

  // Set the HA unique id / MQTT client id from the MAC. This MUST be read AFTER
  // WiFi is initialized - reading it before WiFi.begin() returns uninitialized
  // garbage, giving the device a random client id every boot. That caused
  // broker session churn ("session taken over" / "exceeded timeout") and made
  // the device look like a brand-new device to HA on every reboot, so its
  // entities orphaned and froze.
  byte mac[6];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  // Namespace every entity's unique_id with this device's own MAC-based id
  // (e.g. "04b247842310_fan" instead of a bare "fan"). Without this, any other
  // device - past, present, or a bug reintroducing the old garbage-MAC issue -
  // that ever published discovery under the same bare object_ids can silently
  // shadow this device's entities in Home Assistant, exactly like the stale
  // v1.3.0 ghost identity (0000d3120f80) did to the fan/light/uptime/etc.
  // entities in July 2026. Extended IDs make that class of collision structurally
  // impossible instead of something that has to be manually cleaned up.
  device.enableExtendedUniqueIds();
  DBG_PRINTF("Device unique id (MAC): %02X:%02X:%02X:%02X:%02X:%02X\r\n",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  device.setName("ChromaComfort BT to MQTT Bridge");
  device.setSoftwareVersion("1.5.1");  // ack fix, rx locking, flash diet

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
  rssiSensor.setIcon("mdi:wifi");
  rssiSensor.setName("WiFi RSSI");
  rssiSensor.setUnitOfMeasurement("dBm");
  lastRestartSensor.setIcon("mdi:history");
  lastRestartSensor.setName("Last Restart Reason");
  lastRestartSensor.setValue(lastRestartInfo.c_str());

  rebootButton.setName("Reboot Bridge");
  rebootButton.setIcon("mdi:restart");
  rebootButton.onCommand(onRebootButtonCommand);

  // CRITICAL: PubSubClient's default MQTT buffer is only 256 bytes, and
  // ArduinoHA never enlarges it. The RGB light's discovery (config) payload is
  // 400-600+ bytes, so every discovery publish SILENTLY FAILS at the default
  // size - the device connects and control works (HA->device needs no publish),
  // but discovery/state never reach the broker, so HA never builds the entities.
  // Enlarge the buffer so the biggest discovery message fits. The RGB light's
  // config (rgb + brightness + state/command topics + device block) is the
  // largest single message and exceeds 1024, so use 2048 for safe headroom.
  // Symptom of too-small: the first entities' discovery lands but everything
  // from the RGB light onward silently fails (publish > buffer returns false).
  mqtt.setBufferSize(2048);

  mqtt.begin(BROKER_ADDR, MQTT_USER, MQTT_PASS);
  DBG_PRINTLN("Connected to mqtt");

  // Pump the MQTT client for a few seconds so it fully connects AND flushes its
  // Home Assistant discovery (config) messages on a fresh, healthy connection -
  // BEFORE the (potentially blocking) BLE connect below can starve loop() past
  // the MQTT keepalive. Without this, discovery could get interrupted and HA
  // would never build the entities. Watchdog isn't armed yet, so delay() is safe.
  for (int i = 0; i < 30; i++) {
    mqtt.loop();
    delay(100);
  }
  DBG_PRINTF("MQTT settled (connected=%d), discovery flushed\r\n",
             mqtt.isConnected());

  NimBLEDevice::init("");

  // Attempt the first BLE connection, but DON'T block here if it fails. loop()
  // retries on a timer, so MQTT / Home Assistant come up immediately even when
  // the fan is unreachable at boot (the bridge just shows its entities and
  // (re)connects to the fan in the background).
  DBG_PRINTLN("Attempting initial BLE connection (non-blocking)...");
  connectToBLE();

  // Start the self-heal grace windows now so we don't reboot immediately at boot.
  lastWifiOkAt = lastMqttOkAt = lastBleOkAt = millis();

  // Hardware Task Watchdog (last-resort backstop). If loop() ever stops running
  // entirely - a total freeze the software self-heal can't catch, because the
  // checking code is itself frozen - the silicon forces a chip reset after
  // WDT_TIMEOUT_MS. The ESP32 core 3.x already inits the TWDT, so reconfigure it
  // if init reports it's already running, then subscribe this (loop) task.
  esp_task_wdt_config_t wdtConfig = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  if (esp_task_wdt_init(&wdtConfig) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdtConfig);
  }
  esp_task_wdt_add(NULL);
  DBG_PRINTF("Hardware watchdog armed (%lu ms)\r\n", (unsigned long)WDT_TIMEOUT_MS);
}

int heartbeats_since_ack = 0;
// True from the moment a command's 3x write burst goes out until the first
// ack frame for it comes back. The fan can ack each of the three writes, so
// counting raw A0 40 frames would overcount; this counts acked COMMANDS,
// keeping the Acks sensor comparable to TX.
bool awaitingAck = false;
bool firstRun = true;

void loop() {
  esp_task_wdt_reset();  // pet the hardware watchdog every iteration
  ArduinoOTA.handle();
  mqtt.loop();
  selfHeal();

  if (firstRun) {
    acksSensor.setValue((int32_t)0);
    rxSensor.setValue((int32_t)0);
    txSensor.setValue((int32_t)0);
    errorSensor.setValue((int32_t)0);
    uptimeSensor.setValue((int32_t)0);
    rssiSensor.setValue((int32_t)WiFi.RSSI());
    firstRun = false;
  }

  // Telemetry updates every 2s regardless of BLE state, so uptime keeps climbing
  // even while the fan is unreachable. A frozen uptime then unambiguously means
  // the chip itself is dead (vs. just a dropped BLE link).
  if ((millis() - lastUpdateAt) > 2000) {
    unsigned long uptimeValue = millis() / 1000;
    uptimeSensor.setValue((int32_t)uptimeValue);
    rssiSensor.setValue((int32_t)WiFi.RSSI());
    heartbeats += 1;
    if (tx.getCount() > 0) {
      heartbeats_since_ack += 1;
    }
    lastUpdateAt = millis();
    rxSensor.setValue(rxs);

#if CC_DEBUG
    // Periodic diagnostic heartbeat (every ~10s) so the serial always shows the
    // identity + connection state regardless of BLE traffic.
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag > 10000) {
      lastDiag = millis();
      byte m[6];
      WiFi.macAddress(m);
      Serial.printf(
          "[diag] up=%lus ip=%s mac=%02X%02X%02X%02X%02X%02X rssi=%ddBm wifi=%d "
          "mqtt=%d ble=%d heap=%u\r\n",
          uptimeValue, WiFi.localIP().toString().c_str(), m[0], m[1], m[2],
          m[3], m[4], m[5], (int)WiFi.RSSI(), WiFi.status() == WL_CONNECTED,
          mqtt.isConnected(), bleConnected, (unsigned)ESP.getFreeHeap());
    }
#endif
  }

  if (!bleConnected) {
    // Non-blocking reconnect: retry on a timer instead of delay()-looping, so
    // mqtt.loop() (called above every iteration) keeps Home Assistant alive
    // while the fan is unreachable. selfHeal() reboots if BLE stays down too
    // long. Skip the TX/RX handling until BLE is back.
    if (millis() - lastBleAttemptAt > 5000) {
      lastBleAttemptAt = millis();
      DBG_PRINTLN("BLE not connected, attempting reconnect...");
      connectToBLE();
    }
    return;
  }

  if (tx.getCount() > 0) {
    DBG_PRINTF("It's been %d heartbeats since last ack\r\n",
               heartbeats_since_ack);

    Packet packet;
    tx.peek(&packet);

    DBG_PRINTLN("About to send packet: ");
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
      awaitingAck = true;
    } else {
      DBG_PRINTLN("Write characteristic not available");
    }
  }

  Packet packet;
  bool gotRx = false;
  portENTER_CRITICAL(&rxMux);
  if (rx.getCount() > 0) {
    rx.pop(&packet);
    gotRx = true;
  }
  portEXIT_CRITICAL(&rxMux);

  if (gotRx) {
    rxs += 1;

    // Ack frames are 4 data bytes with control bytes A0 40 (see README). The
    // send path above pops the command from tx as soon as its 3x write burst
    // is out, so ack matching must NOT be gated on tx still being non-empty -
    // that gate is why the Acks sensor sat at 0 while commands visibly worked
    // (and, when a second command WAS queued, it silently popped/dropped it).
    if (packet.len == 4) {
      AckRxCmd* ack = (AckRxCmd*)(packet.data);

      if (ack->control1 == 160 && ack->control2 == 64) {
        if (awaitingAck) {
          awaitingAck = false;
          acks += 1;
          acksSensor.setValue(acks);
        }
        heartbeats_since_ack = 0;
      }
    }

    if (packet.len == 17 && tx.getCount() == 0) {
      StatusRxCmd* status = (StatusRxCmd*)(packet.data);

      if (status->control1 == 160 && status->control2 == 65) {
        byte s = status->status_mask;
        bool isFanOn = ((s >> 7) & 1) == 1;
        bool isLightOn = ((s >> 6) & 1) == 1;
        bool rgbButton = ((s >> 5) & 1) == 1;
        bool isFavoriteColor1Active = ((s >> 3) & 1) == 1;
        int brightness = status->brightness;

#if CC_DEBUG
        // The fan re-sends identical status many times per second; only log it
        // when something actually changes, so the serial stays readable.
        static int lastStatusKey = -1;
        int statusKey = ((int)s << 8) | (brightness & 0xFF);
        if (statusKey != lastStatusKey) {
          lastStatusKey = statusKey;
          Serial.printf(
              "Status: fan=%d white=%d color=%d wallRGB=%d bright=%d\r\n",
              isFanOn, isLightOn, isFavoriteColor1Active, rgbButton, brightness);
        }
#endif

        // The three light modes are mutually exclusive on the fan, so map each
        // status bit to its own entity. The single brightness byte belongs to
        // whichever mode is currently active. (Bits 4/2/1/0 - RGB sweep,
        // favorite color 2, user pattern, reserved - are decoded in README.md
        // but unused here.)
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
    DBG_PRINTLN("Flushing the queue from panic");
    tx.flush();
    heartbeats_since_ack = 0;
    awaitingAck = false;
    panics += 1;
    errorSensor.setValue(panics);
  }
}
