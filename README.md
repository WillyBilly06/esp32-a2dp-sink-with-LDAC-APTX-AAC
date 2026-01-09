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

## 📋 Table of Contents

- [Features](#-features)
- [Supported Codecs](#-supported-codecs)
- [Hardware Requirements](#-hardware-requirements)
- [Installation](#-installation)
- [Building from Source](#️-building-from-source)
- [BLE GATT Services](#-ble-gatt-services)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Credits](#-credits)
- [License](#-license)

---

## ✨ Features

<table>
<tr>
<td width="50%">

### 🎵 Audio
- **Hi-Res LDAC** streaming up to 96kHz/24-bit
- **aptX HD/aptX/aptX-LL** for low latency
- **AAC** for Apple device compatibility
- Real-time **DSP processing**

</td>
<td width="50%">

### 📱 Connectivity
- **BLE GATT** remote control
- Real-time **level meters**
- **3-band EQ** adjustment
- **OTA firmware** updates

</td>
</tr>
</table>

---

## 🎵 Supported Codecs

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
| **MCU** | ESP32-WROVER | Original ESP32 only (not S2/S3/C3) |
| **PSRAM** | 8MB | Required for AAC decoder |
| **Flash** | 8MB | Enables OTA dual partition |
| **DAC** | I2S compatible | PCM5102, MAX98357A, etc. |

> ⚠️ **Important**: Bluetooth Classic A2DP is only supported on the original ESP32 chip.

---

## 📦 Installation

### Quick Start (Pre-built Firmware)

Download the latest release and flash directly to your ESP32:

<details>
<summary><b>🪟 Windows</b></summary>

```cmd
:: Install esptool if not already installed
pip install esptool

:: Flash firmware (replace COM10 with your port)
python -m esptool -p COM10 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

</details>

<details>
<summary><b>🐧 Linux / 🍎 macOS</b></summary>

```bash
# Install esptool if not already installed
pip install esptool

# Flash firmware (replace /dev/ttyUSB0 with your port)
python -m esptool -p /dev/ttyUSB0 -b 460800 --chip esp32 write_flash 0x10000 app-template.bin 0x310000 app-template.bin
```

</details>

> 💡 **Tip**: Replace `COM10` or `/dev/ttyUSB0` with your actual serial port.
> 
> 💡 **Why flash both partitions?** The device uses A/B OTA with two app partitions (`ota_0` and `ota_1`). Flashing both ensures a known-good firmware in both slots. OTA updates alternate between partitions, so if one fails, the device rolls back to the other.

---

## 🛠️ Building from Source

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
<summary><b>🪟 Windows</b></summary>

```cmd
:: Setup ESP-IDF environment
cd D:\esp-idf
install.bat
export.bat

:: Build and flash
cd D:\path\to\project
idf.py set-target esp32
idf.py build
idf.py -p COM10 flash
```

**Flash to both OTA partitions** (recommended for OTA reliability):
```cmd
:: Flash ota_0 (0x10000) and ota_1 (0x310000)
python -m esptool -p COM10 -b 460800 --chip esp32 write_flash 0x10000 build/app-template.bin 0x310000 build/app-template.bin
```

</details>

<details>
<summary><b>🐧 Linux / 🍎 macOS</b></summary>

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

## 🎛️ BLE GATT Services

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

## ⚙️ Configuration

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

## 🐛 Troubleshooting

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

## 🙏 Credits

This project builds upon the excellent work of the open-source community:

| Project | Author | Description |
|:--------|:-------|:------------|
| [esp32-a2dp-sink](https://github.com/cfint/esp32-a2dp-sink) | cfint | Original A2DP sink with codecs |
| [esp-idf](https://github.com/cfint/esp-idf) | cfint | ESP-IDF fork with codec support |
| [libfreeaptx-esp](https://github.com/cfint/libfreeaptx-esp) | cfint | aptX decoder for ESP32 |
| [arduino-fdk-aac](https://github.com/cfint/arduino-fdk-aac) | cfint | AAC decoder |
| [libldac-dec](https://github.com/cfint/libldac-dec) | cfint | LDAC decoder |
| [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) | pschatzmann | A2DP library |
| [arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools) | pschatzmann | Audio processing |

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
