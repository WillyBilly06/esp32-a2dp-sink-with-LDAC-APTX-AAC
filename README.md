<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROVER-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/ESP--IDF-v5.1.4-E7352C?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP-IDF"/>
  <img src="https://img.shields.io/badge/Bluetooth-A2DP-0082FC?style=for-the-badge&logo=bluetooth&logoColor=white" alt="Bluetooth"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<p align="center">
  <b>High-fidelity Bluetooth audio receiver with premium codec support</b><br>
  <sub>LDAC | aptX HD | aptX | aptX-LL | AAC | SBC</sub>
</p>

---

> ## This repository is superseded
>
> Active development has moved to
> **[ESP32-A2DP-SINK-WITH-CODECS-UPDATED](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED)**.
>
> **New projects should start there.** This repository targets ESP-IDF v5.1.4 and is kept
> for reference and for existing builds; it no longer receives updates.
>
> | | This repo | Updated repo |
> |:--|:--|:--|
> | ESP-IDF | v5.1.4 | v5.5.2 |
> | Target module | ESP32-WROVER | ESP32-WROOM |
> | PSRAM | Required | Not required (runs on internal SRAM) |
> | Codecs | LDAC, aptX HD, aptX, aptX-LL, AAC, SBC | adds **Opus** |
> | Bluetooth stack | Stock Bluedroid + codec patches | Patched stack with native A2DP handling |
>
> **Which branch to use over there:**
>
> - [`main`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/main) —
>   ESP32-WROOM, internal SRAM only. Start here unless you need one of the below.
> - [`PSRAM-only`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/PSRAM-only) —
>   the closest match to *this* repo's setup. **If you are running WROVER hardware with PSRAM, use this branch.**
> - [`second-i2s-wm8805-spdif`](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED/tree/second-i2s-wm8805-spdif) —
>   adds a second I2S output with WM8805 S/PDIF.

---

## Table of Contents

- [Features](#features)
- [Supported Codecs](#supported-codecs)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Building from Source](#building-from-source)
- [BLE GATT Services](#ble-gatt-services)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)
- [License](#license)

---

## Features

### Audio Processing

- Hi-Res LDAC streaming up to 96kHz/24-bit
- aptX HD/aptX/aptX-LL for low latency
- AAC for Apple device compatibility
- Real-time DSP processing with optimized biquad filters
- 3-band parametric EQ (bass, mid, treble)
- Stage Presence 3D audio enhancement with bass centering and stereo widening
- Division-free DSP using fast reciprocal approximations for ESP32 optimization

### Visual Feedback

- 16x16 LED Matrix with SPI/DMA driver
- 24 audio-reactive LED effects including spectrum analyzer, VU meter, fire, plasma, starfield, rainbow patterns, and more
- Audio AGC (Automatic Gain Control) with fixed floor normalization for LED reactivity at all volume levels
- Volume/EQ overlay display
- Pairing mode LED animation
- Startup animation with fixed brightness

### Hardware Control

- Quad Rotary Encoder support (Adafruit 5752)
- Volume, Bass, Mid, Treble controls
- LED effect selection via encoder
- Brightness adjustment mode
- Multi-click detection (play/pause/next/prev)

### Connectivity

- BLE GATT remote control
- Real-time level meters
- OTA firmware updates via BLE or Android app
- Encrypted OTA with AES-256-CBC
- WiFi OTA Recovery system

---

## Supported Codecs

| Codec | Bitrate | Sample Rate | Latency | Use Case |
|:------|:-------:|:-----------:|:-------:|:---------|
| LDAC | 990 kbps | 96 kHz | ~200ms | Hi-Res listening |
| aptX HD | 576 kbps | 48 kHz | ~150ms | High quality |
| aptX | 352 kbps | 48 kHz | ~120ms | CD quality |
| aptX-LL | 352 kbps | 48 kHz | ~40ms | Gaming/Video |
| AAC | 256 kbps | 44.1 kHz | ~150ms | Apple devices |
| SBC | 328 kbps | 44.1 kHz | ~200ms | Universal |

---

## Hardware Requirements

| Component | Requirement | Notes |
|:----------|:------------|:------|
| MCU | ESP32-WROVER or ESP32-WROOM | Original ESP32 only (not S2/S3/C3) |
| PSRAM | Optional (8MB recommended) | Enables larger buffers for LDAC/AAC |
| Flash | 8MB | Enables OTA dual partition |
| DAC | I2S compatible | PCM5102, MAX98357A, etc. |
| Encoder | Adafruit Quad Rotary (5752) | Optional - I2C control interface |
| LED Matrix | 16x16 WS2812B | Optional - SPI/DMA driven |

### GPIO Assignments

| GPIO | Function |
|:-----|:---------|
| GPIO 25 | I2S BCLK |
| GPIO 26 | I2S LRC |
| GPIO 27 | I2S DATA |
| GPIO 4 | LED Matrix Data (SPI) |
| GPIO 23 | I2C SDA (Encoder) |
| GPIO 22 | I2C SCL (Encoder) |
| GPIO 18 | Pairing Button |
| GPIO 21 | Power Amp Enable |
| GPIO 5 | Power Amp Mute |

### Quad Rotary Encoder (Adafruit 5752)

| Encoder | Function | Button Action |
|:--------|:---------|:--------------|
| Volume (Green) | Adjust volume | 1-click: Play/Pause, 2-click: Next, 3-click: Previous |
| Bass (Red) | Adjust bass EQ | Click: Enter brightness mode |
| Mid (Blue) | Adjust mid EQ | Click: Enter Bluetooth pairing mode |
| Treble (Yellow) | Adjust treble EQ | Click: Enter effect selection mode |

### PSRAM vs Non-PSRAM Mode

| Mode | Audio Buffers | OTA Buffer | LED Stack | Best For |
|:-----|:-------------|:-----------|:----------|:---------|
| With PSRAM | 48 x 8KB (384KB) | 16KB | 8KB | LDAC 96kHz, AAC |
| Without PSRAM | 16 x 2KB (32KB) | 4KB | 4KB | SBC, aptX |

---

## Installation

### Quick Start (Pre-built Firmware)

Download the latest release and flash directly to your ESP32:

**Windows:**
```cmd
pip install esptool
python -m esptool -p COM10 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

**Linux/macOS:**
```bash
pip install esptool
python -m esptool -p /dev/ttyUSB0 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

Note: Replace COM10 or /dev/ttyUSB0 with your actual serial port.

---

## Building from Source

> **Before you start:** these steps build the ESP-IDF v5.1.4 version and involve manually
> assembling the toolchain and codec libraries. The
> [updated repository](https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED)
> builds against ESP-IDF v5.5.2 with a simpler setup. Follow the steps below only if you
> specifically need the 5.1.4 build.

### Step 1: Clone ESP-IDF with Codec Support

```bash
git clone -b v5.1.4-a2dp-codecs https://github.com/cfint/esp-idf
cd esp-idf
git submodule update --init --recursive
```

### Step 2: Add Codec Libraries

```bash
cd components/bt/host/bluedroid/external/

# aptX decoder
git clone https://github.com/cfint/libfreeaptx-esp libfreeaptx

# AAC decoder
git clone -b idf_component https://github.com/cfint/arduino-fdk-aac arduino-fdk-aac

# LDAC decoder
git clone -b esp32 https://github.com/cfint/libldac-dec libldac-dec
```

### Step 3: Build and Flash

**Windows:**
```cmd
cd D:\esp-idf
install.bat
export.bat

cd D:\path\to\project
idf.py build
idf.py -p COM10 flash
```

**Linux/macOS:**
```bash
cd ~/esp-idf
./install.sh
. ./export.sh

cd ~/path/to/project
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

### Encrypting Firmware for OTA

The firmware supports encrypted OTA updates using AES-256-CBC. To use this feature:

1. Generate a new AES key and update it in `tools/encrypt_firmware.py`
2. Update the same key in `recovery/main/recovery_main.cpp`
3. Configure your Google Drive file ID for firmware hosting

```bash
# Generate a new AES key (save securely)
python tools/encrypt_firmware.py --generate-key

# Encrypt firmware
python tools/encrypt_firmware.py build/app-template.bin --version 1.0.0
```

---

## BLE GATT Services

Control your audio device remotely via Bluetooth Low Energy:

| Service | UUID Prefix | Description |
|:--------|:------------|:------------|
| Level Meters | 0x0042 | Real-time L/R audio levels (0-80) |
| Control | 0x0046 | Play, Pause, Volume, Mute |
| Equalizer | 0x0048 | Bass, Mid, Treble (-12 to +12 dB) |
| Device Name | 0x0050 | Read/write Bluetooth device name |
| OTA Control | 0x0054 | Firmware update control |
| OTA Data | 0x0056 | Firmware binary transfer |
| LED Control | 0x0058 | Effect selection, brightness |

---

## Configuration

Key sdkconfig settings for optimal performance:

```ini
# Flash and Memory
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_SPIRAM_SPEED_80M=y

# CPU Performance
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y

# Bluetooth
CONFIG_BTDM_CTRL_MODE_BTDM=y
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_A2DP_LDAC_DECODER=y
CONFIG_BT_A2DP_APTX_DECODER=y
CONFIG_BT_A2DP_AAC_DECODER=y
```

### DSP Configuration

The DSP processor includes:

- 3-band Parametric EQ: Bass (80Hz), Mid (1kHz), Treble (8kHz)
- Stage Presence 3D: Bass frequencies (<180Hz) centered mono, mids/highs widened 2.2x
- LED Audio Boost: Up to 100x gain for low volume LED reactivity
- Fast Math: Division-free operations using hardware reciprocal approximations

---

## Troubleshooting

**LDAC Low/Medium Quality causes buffer overflow**

Edit `components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_sink.c`:
```c
// Change from:
#define BT_A2DP_SINK_BUF_SIZE   8192
// To:
#define BT_A2DP_SINK_BUF_SIZE   32768
```
Then rebuild with `idf.py fullclean && idf.py build`.

**AAC decoder fails to initialize**

Ensure PSRAM is enabled and running at 80MHz. Check that your ESP32 board has PSRAM (WROVER, not WROOM).

**Bluetooth pairing issues on Linux**

Clear the Bluetooth cache:
```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```
Then re-pair the device.

**LED effects not reactive at low volume**

The firmware includes automatic gain control that boosts audio to 70% minimum for LED effects. If LEDs still don't react, check the LED matrix wiring and GPIO configuration.

---

## Credits

This project builds upon the excellent work of the open-source community:

| Project | Author | Description |
|:--------|:-------|:------------|
| [esp32-a2dp-sink](https://github.com/cfint/esp32-a2dp-sink) | cfint | Original A2DP sink with codecs |
| [esp-idf](https://github.com/cfint/esp-idf) | cfint | ESP-IDF fork with codec support |
| [libfreeaptx-esp](https://github.com/cfint/libfreeaptx-esp) | cfint | aptX decoder for ESP32 |
| [arduino-fdk-aac](https://github.com/cfint/arduino-fdk-aac) | cfint | AAC decoder |
| [libldac-dec](https://github.com/cfint/libldac-dec) | cfint | LDAC decoder |
| [ESP32-A2DP](https://github.com/cfint/ESP32-A2DP/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | A2DP library |
| [arduino-audio-tools](https://github.com/cfint/arduino-audio-tools/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | Audio Processing |

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
