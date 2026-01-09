# ESP-IDF Patches

This directory contains patches to be applied to ESP-IDF v5.1 for optimal LDAC decoding performance.

## btc_a2dp_sink_psram_buffer.patch

**Purpose:** Allocates the A2DP decode buffer from PSRAM instead of internal RAM, and increases the buffer size from 8KB to 64KB.

**Problem Solved:** When using LDAC (especially Low quality mode at 96kHz), the default 8KB decode buffer caused "buffer nearly full, stopping decode" warnings, leading to audio dropouts.

**Changes:**
- Increases `BT_A2DP_SINK_BUF_SIZE` from 8KB to 64KB
- Changes `decode_buf` from a static array to a dynamically allocated pointer
- Allocates the buffer from PSRAM (SPIRAM) if available, with fallback to internal RAM
- Properly frees the buffer during cleanup

### How to Apply

From your ESP-IDF installation directory:

```bash
cd $IDF_PATH
git apply /path/to/esp32-a2dp-sink/esp-idf-patches/btc_a2dp_sink_psram_buffer.patch
```

Or on Windows:
```powershell
cd D:\esp-idf
git apply D:\esp32-a2dp-sink\esp-idf-patches\btc_a2dp_sink_psram_buffer.patch
```

### Requirements

- ESP32 with PSRAM (recommended 4MB+)
- ESP-IDF v5.1
- PSRAM enabled in menuconfig (`CONFIG_SPIRAM=y`)
