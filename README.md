# ChromaComfort BLE → MQTT Bridge

ESP32 firmware that bridges a **Broan ChromaComfort** smart bathroom fan to
**Home Assistant** over MQTT. It talks to the fan over **Bluetooth Low Energy
(BLE)** and exposes the fan, lights, and color controls as native Home Assistant
entities via [ArduinoHA](https://github.com/dawidchyrzynski/arduino-home-assistant)
MQTT discovery.

Because it both **sends commands to** and **receives status notifications from**
the fan, Home Assistant stays in sync even when the fan is operated from its
physical wall switches.

> This is a BLE rewrite of [Taylor Finnell's original classic-Bluetooth
> sketch](https://gist.github.com/taylorfinnell/5349b8085d57836a45be7637055e0692).
> The ChromaComfort's control interface is BLE-only, so the Bluetooth layer was
> rebuilt on [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) and the
> protocol was re-derived from BLE sniffer captures. The packet structures and
> command codes are preserved from the original.

## Features

Exposed in Home Assistant as one MQTT device with these entities:

| Entity | Type | Controls |
|---|---|---|
| **Fan** | switch | Exhaust fan on/off |
| **White Light** | light (dimmable) | The bright white light (1st wall switch) |
| **Color Light** | light (dimmable + RGB) | Custom RGB color ("favorite color") |
| **Wall RGB** | switch | The factory color-cycle mode (2nd wall switch) |
| Uptime / TX / RX / Acks / Errors | sensors | Diagnostics |

The fan's light has **three mutually-exclusive modes** — White, Color, and the
Wall-RGB cycle. Turning one on turns the others off; the firmware reflects this
faithfully, so in Home Assistant those three behave like radio buttons.

## Hardware

- An **ESP32** dev board (developed on a DOIT ESP32 DEVKIT V1 / classic ESP32).
- A Broan **ChromaComfort** fan within BLE range.
- That's it — no wiring. The ESP32 talks to the fan wirelessly and to your
  network over WiFi.

> Note: while the ESP32 is connected to the fan, you won't be able to use the
> fan as a Bluetooth speaker without unplugging the ESP32.

## Software / libraries

Install via the Arduino IDE Library Manager / Boards Manager:

- **ESP32 Arduino core** (developed on 3.3.x)
- **[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) 2.x** (developed on 2.5.0)
- **[ArduinoHA](https://github.com/dawidchyrzynski/arduino-home-assistant)** (home-assistant-integration)
- **[cppQueue](https://github.com/SMFSW/Queue)** (Library Manager name: "Queue")

> NimBLE is required (not the legacy Bluedroid `BLEDevice.h`). See
> [Why NimBLE](#why-nimble) below.

## Setup & flashing

1. **Configure.** Copy the config template and fill in your details:
   ```sh
   cp config.example.h config.h
   ```
   Edit `config.h` with your WiFi credentials, MQTT broker address/credentials,
   and your fan's BLE MAC address (find it with a BLE scanner like nRF Connect).
   `config.h` is gitignored, so your credentials are never committed.

2. **Select the board** in Arduino IDE (e.g. *DOIT ESP32 DEVKIT V1*) and the
   correct serial port.

3. **Upload.** If the upload fails with `Wrong boot mode detected (0x13)`, hold
   the **BOOT** button on the ESP32 down through the entire "Connecting…" phase
   and release it once flashing starts. (Some boards don't auto-reset into the
   bootloader.)

4. The device appears in Home Assistant automatically via MQTT discovery. If you
   add/rename entities later and they don't show up, power-cycle the ESP32 —
   ArduinoHA only republishes discovery on a fresh MQTT connection.

## How it works

The firmware keeps a persistent BLE connection to the fan. Outgoing commands are
queued and written to the fan's command characteristic; incoming status
notifications are parsed and pushed to the matching Home Assistant entities. A
small heartbeat/ack mechanism detects dropped commands and flushes the queue.

---

## Reverse-engineered BLE protocol

Everything below was derived from on-device GATT discovery and BLE sniffer
captures (Nordic sniffer → Wireshark). Documented here so others don't have to
re-derive it.

### ⚠️ The non-obvious bits (read this first — they cost hours)

If the fan connects fine but silently **ignores your commands**, it's almost
certainly one of these. None of them is obvious from any single source:

1. **Send every command 3× with a ~37 ms gap between writes.** The command
   characteristic is *Write Without Response*, so there's no BLE-layer ack and
   the fan drops lone writes. The official app sends each frame **three times,
   ~37 ms apart** — and so must you. This is the single most obscure quirk in
   the whole protocol; one or two writes is unreliable and looks like a totally
   different bug.

2. **The `version` byte must be `0x01`.** The first data byte is a protocol
   version and the fan only honors commands with version **1**. Don't copy the
   `0x05` that appears in the fan's *status replies* into your *commands* — it
   gets silently ignored.

3. **The `speed` byte must be `0` for normal commands.** Leave it `0`; only the
   color-save command (`0x0D`) sets a non-zero speed (`30`). A stray speed value
   changes behavior.

4. **Write to the right characteristic.** The fan exposes a **decoy** service
   with similar-looking UUIDs (`00001016-…`) whose writes do nothing. The real
   command sink is `bb8a27e0-…` (handle `0x001d`) in the `a08f7710-…` service —
   see the table below.

5. **You must subscribe to notifications before commands work.** The fan won't
   start processing commands until a client subscribes to the status
   characteristic (i.e. the CCCD is written). Subscribe first, then send.

### GATT layout

The fan exposes **two** custom services. Watch out:

- `00001016-d102-11e1-9b23-00025b00a5a5` — **decoy.** Contains characteristics
  `1013/1018/1014/1011`. The fan ignores writes here.
- `a08f7710-c37c-11e3-99cc-0228ac012a70` — **the real control service.**

Inside the control service, two characteristics matter:

| Purpose | UUID | Handle | Properties |
|---|---|---|---|
| **Command** (write) | `bb8a27e0-c37c-11e3-b953-0228ac012a70` | `0x001d` | `0x04` Write Without Response |
| **Status** (notify) | `b34ae89e-c37c-11e3-940e-0228ac012a70` | `0x001a` | `0x10` Notify |

The client must **subscribe** to the status characteristic (write the CCCD).
Subscribing is also what makes the fan start processing commands.

Commands are sent as **Write Without Response**, repeated **3×** with a ~37 ms
gap between each (matching the official app; compensates for the unreliable
write-without-response transport).

### Command frame (19 bytes)

```
 3A | 11 | ver | c1 | c2 | type | r | g | b | dim | spd | sw1 | sw2 | dur | t1 | t2 | t3 | t4 | end
 ^hdr ^len  01   00   40                                  01    18
```

| Field | Value / meaning |
|---|---|
| `3A` | header (58) |
| `11` | length (17 data bytes) |
| `ver` | **must be `0x01`** — commands with any other version are silently ignored |
| `c1`/`c2` | control bytes, `0x00` / `0x40` |
| `type` | command code (below) |
| `r g b` | RGB, **raw 0–255, no gamma** |
| `dim` | brightness/dimmer, **0–100** |
| `spd` | effect speed — **keep `0`** for normal commands; only color-save (`0x0D`) uses `30` |
| `sw1`/`sw2` | sweep color values (`0x01` / `0x18` defaults) |
| rest | duration / timers / end (`0x00`) |

> Commands are sent as **Write Without Response, repeated 3× with a ~37 ms gap**
> — see gotcha #1 above. This is essential, not optional.

### Command type codes

| Code | Action |
|---|---|
| `0x01` / `0x02` | Fan on / off |
| `0x03` / `0x04` | White light on / off (`dim` = brightness) |
| `0x0B` / `0x0C` | Activate / deactivate favorite (RGB) color (`dim` = brightness) |
| `0x0D` | Save favorite color (raw `r g b`) |
| `0x05` / `0x06` | Wall RGB (factory cycle) on / off |
| `0x20` / `0x21` / `0x2A` | Custom pattern activate / deactivate / save |

To set a color the app sends `0x0D` (save raw RGB) then `0x0B` (activate). In
practice on this firmware, setting the color while the Color Light is on takes
effect immediately.

### Status notifications

Status frames are notifications with `ver=0x05`, control bytes `A0 41`, followed
by a **status mask** byte, an unknown byte, and a brightness byte. ACKs are
4-byte frames with control bytes `A0 40`.

The status mask is a bitfield of the current state:

| Bit | Meaning |
|---|---|
| 7 | Fan on |
| 6 | White light on |
| 5 | Wall RGB (factory cycle) on |
| 4 | RGB sweep |
| 3 | Favorite color 1 active (Color Light) |
| 2 | Favorite color 2 active |
| 1 | User pattern |

Bits 6, 5, and 3 (the three light modes) are mutually exclusive.

### Why NimBLE

The two characteristics we need share an unusual layout (distinct services, a
decoy service with similar-looking UUIDs). NimBLE-Arduino exposes services and
characteristics as **vectors** and lets you select by UUID/property cleanly,
whereas the legacy Bluedroid `BLEDevice.h` is awkward here. NimBLE is also the
lighter, actively-maintained BLE stack for the ESP32.

---

## Credits

- Original ChromaComfort Bluetooth→MQTT sketch by **Taylor Finnell** —
  https://gist.github.com/taylorfinnell/5349b8085d57836a45be7637055e0692
- BLE rewrite, protocol re-derivation, and Home Assistant entity work in this repo.

## License

[MIT](LICENSE). Built on Taylor Finnell's original work (see above).
