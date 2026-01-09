# ESP32 A2DP Bluetooth Audio Sink with LDAC, APTX & AAC

A high-quality Bluetooth audio receiver (A2DP sink) for ESP32 with support for premium audio codecs and advanced features.

![ESP32](https://img.shields.io/badge/ESP32-WROVER-blue)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.1-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## ✨ Features

- **Premium Codec Support**: LDAC (Hi-Res), aptX/aptX-HD/aptX-LL, AAC
- **BLE GATT Services**: Real-time level meters, EQ control, device settings via Bluetooth LE
- **DSP Processing**: Crossover filters, 3-band parametric EQ, Goertzel-based beat detection
- **OTA Updates**: Over-the-air firmware updates via BLE
- **High Performance**: 96kHz LDAC streaming, optimized for 8MB PSRAM

## 🎵 Supported Codecs

| Codec | Quality | Sample Rate |
|-------|---------|-------------|
| LDAC | Hi-Res (990kbps) | Up to 96kHz |
| aptX HD | High Quality | 48kHz |
| aptX | CD Quality | 48kHz |
| aptX-LL | Low Latency | 48kHz |
| AAC | Good Quality | 44.1kHz |
| SBC | Standard | 44.1kHz |

## 🔧 Hardware Requirements

- **ESP32-WROVER** or similar with **PSRAM** (8MB recommended)
- **8MB Flash** (for OTA dual partition)
- I2S DAC (e.g., PCM5102, MAX98357A)
- Optional: Buttons for control, LED for status

> ⚠️ **Note**: Only the original ESP32 (not S2/S3/C3) supports Bluetooth Classic A2DP.

## 📦 Installation

### Prerequisites

1. Install ESP-IDF v5.1.4 with A2DP codec support:
   ```bash
   git clone -b v5.1.4-a2dp-codecs https://github.com/cfint/esp-idf
   ```

2. Add codec libraries to ESP-IDF:
   ```bash
   cd esp-idf/components/bt/host/bluedroid/external/
   git clone https://github.com/cfint/libfreeaptx-esp libfreeaptx
   git clone -b idf_component https://github.com/cfint/arduino-fdk-aac arduino-fdk-aac
   git clone -b esp32 https://github.com/cfint/libldac-dec libldac-dec
   ```

### Build & Flash

```bash
. ~/esp-idf/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## 🎛️ BLE GATT Services

Connect via BLE to control your audio device:

| Service | Function |
|---------|----------|
| Level Meters | Real-time L/R audio levels (0-80) |
| Control | Play/Pause, Volume, Mute |
| Equalizer | Bass, Mid, Treble adjustment |
| Device Name | Change Bluetooth name |
| OTA Update | Firmware updates via BLE |

## ⚙️ Configuration

Key settings in `sdkconfig`:
- Flash: 8MB @ 80MHz QIO mode
- PSRAM: 8MB @ 80MHz
- CPU: 240MHz
- Bluetooth: Classic + BLE dual mode

## 🐛 Known Issues & Fixes

### LDAC Low/Medium Quality Buffer Overflow
If you experience audio issues with LDAC at lower quality settings, patch ESP-IDF:

In `components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_sink.c`, change:
```c
#define BT_A2DP_SINK_BUF_SIZE   32768  // was 8192 when APTX enabled
```

## 🙏 Credits

This project is based on the excellent work by **cfint**:
- Original Project: [cfint/esp32-a2dp-sink](https://github.com/cfint/esp32-a2dp-sink)
- ESP-IDF Fork: [cfint/esp-idf](https://github.com/cfint/esp-idf) (v5.1.4-a2dp-codecs branch)
- Codec Libraries: [libfreeaptx-esp](https://github.com/cfint/libfreeaptx-esp), [arduino-fdk-aac](https://github.com/cfint/arduino-fdk-aac), [libldac-dec](https://github.com/cfint/libldac-dec)

Additional libraries:
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) by pschatzmann
- [arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools) by pschatzmann

## 📝 License

MIT License - See [LICENSE](LICENSE) for details.

---

**Made with ❤️ for Hi-Fi Bluetooth Audio**
