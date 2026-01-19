<p align="center">
  <img src="https://img.shields.io/badge/ESP32-WROVER-000000?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/ESP--IDF-v5.1.4-E7352C?style=for-the-badge&logo=espressif&logoColor=white" alt="ESP-IDF"/>
  <img src="https://img.shields.io/badge/Bluetooth-A2DP-0082FC?style=for-the-badge&logo=bluetooth&logoColor=white" alt="Bluetooth"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="License"/>
</p>

<p align="center">
  <b>High-fidelity Bluetooth audio receiver with premium codec support</b><br>
  <sub>LDAC • aptX HD • aptX • aptX-LL • AAC • SBC</sub>
</p>

---

##  Table of Contents

- [Features](#features)
- [Supported Codecs](#supported-codecs)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Building from Source](#️building-from-source)
- [BLE GATT Services](#ble-gatt-services)
- [Configuration](#configuration)
- [WiFi OTA Recovery](#wifi-ota-recovery)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)
- [License](#license)

---

##  Features

<table>
<tr>
<td width="50%">

###  Audio
- **Hi-Res LDAC** streaming up to 96kHz/24-bit
- **aptX HD/aptX/aptX-LL** for low latency
- **AAC** for Apple device compatibility
- Real-time **DSP processing**
- **TWS stereo pairing** via ESP-NOW

</td>
<td width="50%">

###  Connectivity
- **BLE GATT** remote control
- Real-time **level meters**
- **3-band EQ** adjustment
- **OTA firmware** updates

</td>
</tr>
<tr>
<td width="50%">

###  Hardware Control
- **Quad Rotary Encoder** support (Adafruit 5752)
- Volume, Bass, Mid, Treble controls
- **LED effect selection** via encoder
- **Brightness adjustment** mode
- Multi-click detection (play/pause/next/prev)

</td>
<td width="50%">

###  Visual Feedback
- **16×16 LED Matrix** effects
- 24 audio-reactive LED effects
- Volume/EQ overlay display
- **Pairing mode** LED animation
- Effect preview via encoder

</td>
</tr>
</table>

---

##  Supported Codecs

| Codec | Bitrate | Sample Rate | Latency | Use Case |
|:------|:-------:|:-----------:|:-------:|:---------|
| **LDAC** | 990 kbps | 96 kHz | ~200ms | Hi-Res listening |
| **aptX HD** | 576 kbps | 48 kHz | ~150ms | High quality |
| **aptX** | 352 kbps | 48 kHz | ~120ms | CD quality |
| **aptX-LL** | 352 kbps | 48 kHz | ~40ms | Gaming/Video |
| **AAC** | 256 kbps | 44.1 kHz | ~150ms | Apple devices |
| **SBC** | 328 kbps | 44.1 kHz | ~200ms | Universal |

---

## 🔧 Hardware Requirements

| Component | Requirement | Notes |
|:----------|:------------|:------|
| **MCU** | ESP32-WROVER or ESP32-WROOM | Original ESP32 only (not S2/S3/C3) |
| **PSRAM** | Optional (8MB recommended) | Enables larger buffers for LDAC/AAC |
| **Flash** | 8MB | Enables OTA dual partition |
| **DAC** | I2S compatible | PCM5102, MAX98357A, etc. |
| **Encoder** | Adafruit Quad Rotary (5752) | Optional - I2C control interface |
| **LED Matrix** | 16×16 WS2812B | Optional - SPI/DMA driven |

### TWS (True Wireless Stereo) Mode

The firmware supports TWS operation where two ESP32 speakers work together as a stereo pair:

| Role | Function | Description |
|:-----|:---------|:------------|
| **Primary** | A2DP Receiver | Connects to phone, receives audio, sends one channel to Secondary |
| **Secondary** | ESP-NOW Receiver | Receives audio from Primary via ESP-NOW, outputs the other channel |

**How it works:**
1. Primary receives stereo audio from phone via A2DP (Bluetooth)
2. Primary plays one channel (e.g., Left) through its speaker
3. Primary sends the other channel (e.g., Right) to Secondary via ESP-NOW (~1-3ms latency)
4. A sync delay buffer on Primary ensures both speakers play in sync

**Configuration** (via `idf.py menuconfig` → TWS Configuration):

| Setting | Options | Description |
|:--------|:--------|:------------|
| **Enable TWS** | On/Off | Enable TWS mode |
| **Role** | Primary/Secondary | Device role in TWS pair |
| **Channel** | Left/Right | Which channel this device plays |
| **Sync Delay** | 10-200ms | Buffer delay for sync (default: 50ms) |
| **Force SBC** | On/Off | Force SBC codec for timing consistency |
| **Peer MAC** | AA:BB:CC:DD:EE:FF | Pre-configured peer MAC (optional) |

**Building TWS Firmware:**
```bash
# Build Primary speaker
idf.py menuconfig   # Enable TWS → Role: Primary → Channel: Left
idf.py build
idf.py flash

# Build Secondary speaker (on different ESP32)
idf.py menuconfig   # Enable TWS → Role: Secondary → Channel: Right
idf.py build
idf.py flash
```

> ⚠️ **Note**: TWS mode requires WiFi for ESP-NOW. The ESP32 uses WiFi Station mode internally (no router connection needed).

> 💡 **Tip**: For best sync, use the same sync delay on both devices and enable "Force SBC" to avoid codec timing variations.

### Quad Rotary Encoder (Adafruit 5752)

The firmware supports the [Adafruit I2C QT Quad Rotary Encoder](https://www.adafruit.com/product/5752) for hardware controls:

| Encoder | Function | Button Action |
|:--------|:---------|:--------------|
| **Volume** (Green) | Adjust volume | 1-click: Play/Pause, 2-click: Next, 3-click: Previous |
| **Bass** (Red) | Adjust bass EQ | Click: Enter brightness mode, rotate to adjust |
| **Mid** (Blue) | Adjust mid EQ | Click: Enter Bluetooth pairing mode |
| **Treble** (Yellow) | Adjust treble EQ | Click: Enter effect selection mode, rotate to preview |

**Wiring:**
| Encoder Pin | ESP32 GPIO |
|:------------|:-----------|
| SDA | GPIO 23 |
| SCL | GPIO 22 |
| VIN | 3.3V |
| GND | GND |

### PSRAM vs Non-PSRAM Mode

The firmware supports both ESP32-WROVER (with PSRAM) and ESP32-WROOM (without PSRAM):

| Mode | Audio Buffers | OTA Buffer | LED Stack | Best For |
|:-----|:-------------|:-----------|:----------|:---------|
| **With PSRAM** | 48 × 8KB (384KB) | 16KB | 8KB | LDAC 96kHz, AAC |
| **Without PSRAM** | 16 × 2KB (32KB) | 4KB | 4KB | SBC, aptX |

To switch modes, run `idf.py menuconfig` and navigate to:
`ESP32 Bluetooth Speaker → Audio Buffer Configuration → Use PSRAM for audio buffers`

> ⚠️ **Note**: Without PSRAM, LDAC 96kHz and AAC may experience buffer underruns. SBC and aptX work reliably.

> ⚠️ **Important**: Bluetooth Classic A2DP is only supported on the original ESP32 chip.

---

##  Installation

### Quick Start (Pre-built Firmware)

Download the latest release and flash directly to your ESP32:

<details>
<summary><b> Windows</b></summary>

```cmd
:: Install esptool if not already installed
pip install esptool

:: Flash firmware (replace COM10 with your port)
python -m esptool -p COM10 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

</details>

<details>
<summary><b> Linux /  macOS</b></summary>

```bash
# Install esptool if not already installed
pip install esptool

# Flash firmware (replace /dev/ttyUSB0 with your port)
python -m esptool -p /dev/ttyUSB0 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

</details>

>  **Tip**: Replace `COM10` or `/dev/ttyUSB0` with your actual serial port.
> 
>  **Why flash both partitions?** The device uses A/B OTA with two app partitions (`ota_0` and `ota_1`). Flashing both ensures a known-good firmware in both slots. OTA updates alternate between partitions, so if one fails, the device rolls back to the other.

---

##  Building from Source

If you want to customize the firmware or build your own, follow these steps:

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

### Step 3: Build & Flash

<details>
<summary><b> Windows</b></summary>

```cmd
:: Setup ESP-IDF environment
cd D:\esp-idf
install.bat
export.bat

:: Build and flash
cd D:\path\to\project
idf.py build
idf.py -p COMXX flash
:: XX will be the COM of the current connected ESP32 on your computer
```

**Flash to both OTA partitions** (recommended for OTA reliability):
```cmd
:: Flash ota_0 (0x10000) and ota_1 (0x310000)
python -m esptool -p COM10 -b 460800 --chip esp32 write_flash 0x10000 build/app-template.bin 0x310000 build/app-template.bin
```

</details>

<details>
<summary><b> Linux /  macOS</b></summary>

```bash
# Setup ESP-IDF environment
cd ~/esp-idf
./install.sh
. ./export.sh

# Build and flash
cd ~/path/to/project
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

**Flash to both OTA partitions** (recommended for OTA reliability):
```bash
# Flash ota_0 (0x10000) and ota_1 (0x310000)
python -m esptool -p /dev/ttyUSB0 -b 460800 --chip esp32 write_flash 0x10000 build/app-template.bin 0x310000 build/app-template.bin
```

</details>

---

##  BLE GATT Services

Control your audio device remotely via Bluetooth Low Energy:

| Service | UUID Prefix | Description |
|:--------|:------------|:------------|
| **Level Meters** | `0x0042` | Real-time L/R audio levels (0-80) |
| **Control** | `0x0046` | Play, Pause, Volume, Mute |
| **Equalizer** | `0x0048` | Bass, Mid, Treble (-12 to +12 dB) |
| **Device Name** | `0x0050` | Read/write Bluetooth device name |
| **OTA Control** | `0x0054` | Firmware update control |
| **OTA Data** | `0x0056` | Firmware binary transfer |

---

##  Configuration

Key `sdkconfig` settings for optimal performance:

```ini
# Flash & Memory
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_SPIRAM_SPEED_80M=y

# CPU Performance
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y

# Bluetooth
CONFIG_BTDM_CTRL_MODE_BTDM=y          # Classic + BLE
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_A2DP_LDAC_DECODER=y
CONFIG_BT_A2DP_APTX_DECODER=y
CONFIG_BT_A2DP_AAC_DECODER=y
```

---

## 📡 WiFi OTA Recovery

The firmware includes a **WiFi-based recovery system** that can restore your device if the main firmware becomes corrupted or unbootable. This is a safety net for OTA updates gone wrong.

### How It Works

1. **Recovery Partition**: A small recovery firmware lives at `0x10000` (same as `ota_0`)
2. **WiFi Provisioning**: On first boot, scan the QR code or connect to the ESP32's WiFi AP
3. **Firmware Download**: Recovery fetches encrypted firmware from a secure URL
4. **AES Decryption**: Firmware is decrypted using AES-256-CBC before flashing
5. **Auto-boot**: After successful flash, the device reboots into the main firmware

### Building the Recovery Firmware

```bash
# Navigate to recovery folder
cd recovery

# Configure WiFi credentials (optional - can also use provisioning)
idf.py menuconfig
# → Component config → Recovery Configuration

# Build recovery firmware
idf.py build

# Flash recovery to the device
idf.py -p COM10 flash
```

### Encrypting Firmware for OTA

The `tools/encrypt_firmware.py` script encrypts your firmware for secure distribution:

```bash
# Generate a new AES key (do this once, save it securely!)
python tools/encrypt_firmware.py --generate-key

# Encrypt a firmware binary
python tools/encrypt_firmware.py build/app-template.bin --version 1.0.0
```

This creates:
- `ota_releases/app-template_v1.0.0.bin.enc` - Encrypted firmware
- `ota_releases/latest.txt` - Version info file

> ⚠️ **Important**: The AES key in `encrypt_firmware.py` must match the key in `recovery/main/recovery_main.cpp`. Keep your key secret!

### GPIO Assignments (Recovery Mode)

| GPIO | Function | Notes |
|:-----|:---------|:------|
| GPIO 2 | Status LED | Built-in LED on most boards |
| GPIO 23 | I2C SDA | Shared with main firmware |
| GPIO 22 | I2C SCL | Shared with main firmware |

### LED Status Indicators

| Pattern | Meaning |
|:--------|:--------|
| Slow blink (1s) | Waiting for WiFi provisioning |
| Fast blink (200ms) | Connected, downloading firmware |
| Solid ON | Flashing firmware |
| 3 quick blinks | Success, rebooting |

---

##  Troubleshooting

<details>
<summary><b>LDAC Low/Medium Quality causes buffer overflow</b></summary>

**Problem**: Audio cuts out or errors appear when using LDAC at lower quality settings.

**Solution**: Patch ESP-IDF to increase the decode buffer size.

Edit `components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_sink.c`:

```c
// Change from:
#define BT_A2DP_SINK_BUF_SIZE   8192

// To:
#define BT_A2DP_SINK_BUF_SIZE   32768
```

Then rebuild with `idf.py fullclean && idf.py build`.

</details>

<details>
<summary><b>AAC decoder fails to initialize</b></summary>

**Problem**: AAC codec doesn't work or causes memory errors.

**Solution**: Ensure PSRAM is enabled and running at 80MHz. Check that your ESP32 board has PSRAM (WROVER, not WROOM).

</details>

<details>
<summary><b>Bluetooth pairing issues on Linux</b></summary>

**Problem**: Codecs don't appear after changing configuration.

**Solution**: Clear the Bluetooth cache:
```bash
sudo rm -rf /var/lib/bluetooth/<adapter-mac>/cache/<device-mac>
```
Then re-pair the device.

</details>

---

##  Credits

This project builds upon the excellent work of the open-source community:

| Project | Author | Description |
|:--------|:-------|:------------|
| [esp32-a2dp-sink](https://github.com/cfint/esp32-a2dp-sink) | cfint | Original A2DP sink with codecs |
| [esp-idf](https://github.com/cfint/esp-idf) | cfint | ESP-IDF fork with codec support |
| [libfreeaptx-esp](https://github.com/cfint/libfreeaptx-esp) | cfint | aptX decoder for ESP32 |
| [arduino-fdk-aac](https://github.com/cfint/arduino-fdk-aac) | cfint | AAC decoder |
| [libldac-dec](https://github.com/cfint/libldac-dec) | cfint | LDAC decoder |
| [ESP32-A2DP](https://github.com/cfint/ESP32-A2DP/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | A2DP library |
| [arduino-audio-tools](https://github.com/cfint/arduino-audio-tools/tree/v5.1-a2dp_codecs) | cfint/pschatzmann | A2DP/Audio Processing |

---

##  License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
