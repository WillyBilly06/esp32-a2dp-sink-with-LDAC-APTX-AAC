// -----------------------------------------------------------
// ESP32 A2DP Sink with LDAC/aptX/AAC + BLE GATT + DSP
// Main entry point - coordinates all modules
// -----------------------------------------------------------

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_bt_device.h"
#include "BluetoothA2DPSink.h"

// Modular components
#include "config/app_config.h"
#include "dsp/dsp_processor.h"
#include "storage/nvs_settings.h"
#include "audio/i2s_output.h"
#include "audio/audio_pipeline.h"
#include "ble/ble_gatt.h"
#include "idf_update.h"

static const char* TAG = "Main";

// -----------------------------------------------------------
// Global instances (no raw global state - encapsulated in classes)
// -----------------------------------------------------------
static NVSSettings     g_settings;
static DSPProcessor    g_dsp;
static I2SOutput       g_i2s;
static AudioPipeline   g_pipeline;
static BleGattService  g_ble;
static BluetoothA2DPSink g_a2dp;
static IdfUpdate       g_update;

// Audio state
static volatile uint8_t  g_bitsPerSample = 16;
static volatile uint8_t  g_channels = 2;
static volatile uint32_t g_sampleRate = APP_I2S_DEFAULT_SAMPLE_RATE;
static volatile bool     g_otaActive = false;

// Beat detection state
static float smooth30_dB = -60.0f;
static float smooth60_dB = -60.0f;
static float smooth100_dB = -60.0f;

// -----------------------------------------------------------
// Control helpers
// -----------------------------------------------------------
static uint8_t getControlByte() {
    uint8_t b = 0;
    if (g_dsp.isBassBoostEnabled()) b |= 0x01;
    if (g_dsp.isChannelFlipEnabled()) b |= 0x02;
    if (g_dsp.isBypassEnabled()) b |= 0x04;
    return b;
}

static void applyControlByte(uint8_t b) {
    g_dsp.setBassBoost(b & 0x01);
    g_dsp.setChannelFlip(b & 0x02);
    g_dsp.setBypass(b & 0x04);
    g_settings.saveControl(b & 0x01, b & 0x02, b & 0x04);
    g_ble.updateControl(getControlByte());
}

static void applyEq(int8_t bass, int8_t mid, int8_t treble) {
    g_dsp.setEQ(bass, mid, treble, g_sampleRate);
    g_settings.saveEQ(bass, mid, treble);
    g_ble.updateEq(bass, mid, treble);
}

// -----------------------------------------------------------
// BLE callbacks
// -----------------------------------------------------------
static void onBleControl(uint8_t ctrl) {
    applyControlByte(ctrl);
    ESP_LOGI(TAG, "BLE control: 0x%02x", ctrl);
}

static void onBleEq(int8_t bass, int8_t mid, int8_t treble) {
    applyEq(bass, mid, treble);
    ESP_LOGI(TAG, "BLE EQ: %d/%d/%d", bass, mid, treble);
}

static void onBleName(const char* name, size_t len) {
    g_settings.saveDeviceName(name);
    // Update Classic Bluetooth (A2DP) device name
    esp_bt_dev_set_device_name(name);
    // BLE advertising name is updated by ble_gatt
    ESP_LOGI(TAG, "BLE name changed: %s", name);
}

static volatile uint32_t g_otaReceived = 0;
static volatile uint32_t g_otaTotalSize = 0;

// Parse OTA control command - supports both binary and ASCII protocols
// Binary: 0x01 + 4-byte size (BEGIN), 0x03 (END), 0x04 (ABORT)
// ASCII: "BEGIN:<size>", "END", "ABORT"
static void onBleOtaCtrl(const uint8_t* data, size_t len) {
    ESP_LOGI(TAG, "OTA CTRL received: len=%u, first=0x%02X", (unsigned)len, len > 0 ? data[0] : 0);
    if (len < 1) return;
    
    // Check for ASCII protocol (starts with 'B', 'E', or 'A')
    if (data[0] == 'B' && len >= 6 && memcmp(data, "BEGIN:", 6) == 0) {
        // Parse ASCII: "BEGIN:<size>"
        char sizeBuf[16] = {0};
        size_t copyLen = (len - 6 < sizeof(sizeBuf) - 1) ? (len - 6) : (sizeof(sizeBuf) - 1);
        memcpy(sizeBuf, data + 6, copyLen);
        uint32_t size = (uint32_t)atoi(sizeBuf);
        
        ESP_LOGI(TAG, "OTA BEGIN (ASCII): %u bytes", (unsigned)size);
        g_otaActive = true;
        g_otaReceived = 0;
        g_otaTotalSize = size;
        if (!g_update.begin(size)) {
            ESP_LOGE(TAG, "OTA begin failed: %s", g_update.errorString());
            g_ble.notifyOtaCtrl("BEGIN_ERR");
            g_otaActive = false;
        } else {
            ESP_LOGI(TAG, "OTA begin OK, waiting for data...");
            g_ble.notifyOtaCtrl("BEGIN_OK");
        }
        return;
    }
    
    if (data[0] == 'E' && len >= 3 && memcmp(data, "END", 3) == 0) {
        ESP_LOGI(TAG, "OTA END (ASCII): received %u bytes total", (unsigned)g_otaReceived);
        if (g_update.end(true)) {
            g_ble.notifyOtaCtrl("END_OK");
            ESP_LOGI(TAG, "OTA complete, restarting...");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "OTA end failed: %s", g_update.errorString());
            g_ble.notifyOtaCtrl("END_ERR");
        }
        g_otaActive = false;
        return;
    }
    
    if (data[0] == 'A' && len >= 5 && memcmp(data, "ABORT", 5) == 0) {
        ESP_LOGW(TAG, "OTA ABORT (ASCII)");
        g_otaActive = false;
        g_otaReceived = 0;
        g_ble.notifyOtaCtrl("ABORT_OK");
        return;
    }
    
    // Binary protocol fallback
    uint8_t cmd = data[0];
    
    if (cmd == 0x01 && len >= 5) { // BEGIN
        uint32_t size = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        ESP_LOGI(TAG, "OTA BEGIN (binary): %u bytes", (unsigned)size);
        g_otaActive = true;
        g_otaReceived = 0;
        g_otaTotalSize = size;
        if (!g_update.begin(size)) {
            ESP_LOGE(TAG, "OTA begin failed: %s", g_update.errorString());
            g_ble.notifyOtaCtrl("BEGIN_ERR");
            g_otaActive = false;
        } else {
            ESP_LOGI(TAG, "OTA begin OK, waiting for data...");
            g_ble.notifyOtaCtrl("BEGIN_OK");
        }
    } else if (cmd == 0x03) { // END
        ESP_LOGI(TAG, "OTA END (binary): received %u bytes total", (unsigned)g_otaReceived);
        if (g_update.end(true)) {
            g_ble.notifyOtaCtrl("END_OK");
            ESP_LOGI(TAG, "OTA complete, restarting...");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else {
            ESP_LOGE(TAG, "OTA end failed: %s", g_update.errorString());
            g_ble.notifyOtaCtrl("END_ERR");
        }
        g_otaActive = false;
    } else if (cmd == 0x04) { // ABORT
        ESP_LOGW(TAG, "OTA ABORT (binary)");
        g_otaActive = false;
        g_otaReceived = 0;
    } else {
        ESP_LOGW(TAG, "OTA CTRL unknown cmd: 0x%02X (char='%c')", cmd, (cmd >= 32 && cmd < 127) ? cmd : '?');
    }
}

static void onBleOtaData(const uint8_t* data, size_t len) {
    if (!g_otaActive || len == 0) return;
    g_otaReceived += len;
    size_t written = g_update.write(data, len);
    
    // Update progress on single line using carriage return
    uint8_t pct = (g_otaTotalSize > 0) ? (uint8_t)((uint64_t)g_otaReceived * 100 / g_otaTotalSize) : 0;
    printf("\rOTA: %3u%% (%u / %u bytes)    ", pct, (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
    fflush(stdout);
}

// -----------------------------------------------------------
// A2DP callbacks
// -----------------------------------------------------------

// Helper to get codec name from type
static const char* getCodecName(esp_a2d_mct_t type) {
    switch (type) {
        case ESP_A2D_MCT_SBC:      return "SBC";
        case ESP_A2D_MCT_M12:      return "MPEG-1/2";
        case ESP_A2D_MCT_M24:      return "AAC";
        case ESP_A2D_MCT_ATRAC:    return "ATRAC";
        case ESP_A2D_MCT_NON_A2DP: return "Vendor Codec (see CODEC_CONFIG log above for details)";
        default:                   return "Unknown";
    }
}

static void onCodecConfig(uint32_t rate, uint8_t bps, uint8_t channels) {
    if (rate == 0) rate = 44100;
    
    // Get codec type (the library's codec_config.c already logs vendor ID details)
    esp_a2d_mct_t codecType = g_a2dp.get_audio_type();
    const char* codecName = getCodecName(codecType);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CODEC CONFIGURATION RECEIVED");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Codec type: %s (0x%02X)", codecName, codecType);
    ESP_LOGI(TAG, "  Sample rate: %u Hz", (unsigned)rate);
    ESP_LOGI(TAG, "  Bits/sample: %u", (unsigned)bps);
    ESP_LOGI(TAG, "  Channels: %u", (unsigned)channels);
    
    // Note: For vendor codecs, check CODEC_CONFIG log above which shows:
    // - "configure LDAC codec" for LDAC (Vendor ID: 0x012D, Codec ID: 0x00AA)
    // - "configure aptX codec" for aptX (Vendor ID: 0x004F, Codec ID: 0x0001)
    // - "configure aptX-HD codec" for aptX-HD (Vendor ID: 0x00D7, Codec ID: 0x0024)
    
    ESP_LOGI(TAG, "========================================");
    
    g_sampleRate = rate;
    g_bitsPerSample = bps;
    g_channels = channels;
    g_i2s.updateClock(rate);
    g_dsp.setSampleRate(rate);
}

static void onStreamData(const uint8_t* data, uint32_t len) {
    g_pipeline.enqueue(data, len, g_bitsPerSample, g_channels);
}

static void onConnectionState(esp_a2d_connection_state_t state, void* user) {
    const char* stateStr = "Unknown";
    switch (state) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED: stateStr = "DISCONNECTED"; break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:   stateStr = "CONNECTING"; break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:    stateStr = "CONNECTED"; break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: stateStr = "DISCONNECTING"; break;
    }
    
    ESP_LOGI(TAG, ">>> A2DP Connection: %s", stateStr);
    
    if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        ESP_LOGW(TAG, "A2DP disconnected - waiting for phone to reconnect with new codec...");
        g_pipeline.clear();
        g_i2s.updateClock(APP_I2S_DEFAULT_SAMPLE_RATE);
        g_dsp.setSampleRate(APP_I2S_DEFAULT_SAMPLE_RATE);
    } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        ESP_LOGI(TAG, "A2DP connected - codec will be configured shortly...");
    }
}

static void onAudioState(esp_a2d_audio_state_t state, void* user) {
    const char* stateStr = "Unknown";
    switch (state) {
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND: stateStr = "REMOTE_SUSPEND"; break;
        case ESP_A2D_AUDIO_STATE_STOPPED:        stateStr = "STOPPED"; break;
        case ESP_A2D_AUDIO_STATE_STARTED:        stateStr = "STARTED"; break;
    }
    
    ESP_LOGI(TAG, ">>> A2DP Audio State: %s", stateStr);
    
    if (state == ESP_A2D_AUDIO_STATE_STOPPED || state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) {
        smooth30_dB = smooth60_dB = smooth100_dB = -60.0f;
        g_pipeline.clear();
    }
}

// -----------------------------------------------------------
// Audio TX task - highest priority for smooth playback
// -----------------------------------------------------------
static void audioTxTask(void* arg) {
    while (true) {
        g_pipeline.processBuffer(g_dsp, g_i2s);
    }
}

// -----------------------------------------------------------
// Button handling task
// -----------------------------------------------------------
static void buttonsTask(void* arg) {
    bool lastBtn1 = true, btn1Pressed = false;
    uint32_t debounce1 = 0, pressStart1 = 0;
    bool lastBtn2 = true, btn2State = true;
    uint32_t debounce2 = 0;

    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        
        bool r1 = gpio_get_level((gpio_num_t)APP_BUTTON1_GPIO);
        if (r1 != lastBtn1) debounce1 = now;
        if ((now - debounce1) > 25) {
            if (!btn1Pressed && r1 == 0) {
                btn1Pressed = true;
                pressStart1 = now;
            } else if (btn1Pressed && r1 == 1) {
                btn1Pressed = false;
                if ((now - pressStart1) < 1000) {
                    g_dsp.setBassBoost(!g_dsp.isBassBoostEnabled());
                } else {
                    g_dsp.setChannelFlip(!g_dsp.isChannelFlipEnabled());
                }
                g_settings.saveControl(g_dsp.isBassBoostEnabled(), 
                                       g_dsp.isChannelFlipEnabled(), 
                                       g_dsp.isBypassEnabled());
                g_ble.updateControl(getControlByte());
            }
        }
        lastBtn1 = r1;

        bool r2 = gpio_get_level((gpio_num_t)APP_BUTTON2_GPIO);
        if (r2 != lastBtn2) debounce2 = now;
        if ((now - debounce2) > 25 && r2 != btn2State) {
            btn2State = r2;
            if (r2 == 1) {
                g_dsp.setBypass(!g_dsp.isBypassEnabled());
                g_settings.saveControl(g_dsp.isBassBoostEnabled(),
                                       g_dsp.isChannelFlipEnabled(),
                                       g_dsp.isBypassEnabled());
                g_ble.updateControl(getControlByte());
            }
        }
        lastBtn2 = r2;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// -----------------------------------------------------------
// Beat detection + levels task
// -----------------------------------------------------------
static void beatTask(void* arg) {
    uint32_t lastLevelMs = 0;
    uint32_t lastBeatMs = 0;
    bool flashActive = false;
    uint32_t flashOffMs = 0;
    float bassSmooth = 0, bassAvg = 0;

    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (!g_otaActive) {
            float bass = g_dsp.getGoertzel60Lin() + g_dsp.getGoertzel100Lin();

            if (bass >= APP_BASS_MIN_LEVEL) {
                bassSmooth += APP_BASS_SMOOTH_ALPHA * (bass - bassSmooth);
                bassAvg += APP_BASS_AVG_ALPHA * (bassSmooth - bassAvg);
                
                float ratio = (bassAvg > 1e-6f) ? (bassSmooth / (bassAvg + 1e-6f)) : 0;
                if (ratio > APP_BASS_RATIO_THRESH && (now - lastBeatMs) > APP_BEAT_MIN_INTERVAL_MS) {
                    gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 1);
                    flashActive = true;
                    flashOffMs = now + APP_BEAT_FLASH_DURATION_MS;
                    lastBeatMs = now;
                }
            } else {
                bassSmooth *= 0.9f;
                bassAvg *= 0.999f;
            }

            if (flashActive && now >= flashOffMs) {
                gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);
                flashActive = false;
            }

            // Update BLE levels every 50ms
            if ((now - lastLevelMs) >= APP_LEVELS_UPDATE_MS) {
                lastLevelMs = now;
                
                bool audioStopped = (now - g_pipeline.getLastProcessMs()) > 100;
                if (audioStopped) {
                    smooth30_dB = smooth30_dB * 0.85f + (-60.0f) * 0.15f;
                    smooth60_dB = smooth60_dB * 0.85f + (-60.0f) * 0.15f;
                    smooth100_dB = smooth100_dB * 0.85f + (-60.0f) * 0.15f;
                } else {
                    // Use fast peak meter for responsive level display
                    // Less smoothing = more reactive to beats
                    smooth30_dB += 0.6f * (g_dsp.getPeakDB(0) - smooth30_dB);
                    smooth60_dB += 0.6f * (g_dsp.getPeakDB(1) - smooth60_dB);
                    smooth100_dB += 0.6f * (g_dsp.getPeakDB(2) - smooth100_dB);
                }

                // Convert dB to display position (0-80)
                // Range: -60dB to 0dB maps to 0-80
                auto dbToPos = [](float dB) -> int {
                    // Clamp to range
                    if (dB < -60.0f) dB = -60.0f;
                    if (dB > 0.0f) dB = 0.0f;
                    // Map -60..0 to 0..80
                    int v = (int)roundf((dB + 60.0f) * (80.0f / 60.0f));
                    return (v < 0) ? 0 : ((v > 80) ? 80 : v);
                };

                if (g_ble.isConnected()) {
                    g_ble.updateLevels(dbToPos(smooth30_dB), dbToPos(smooth60_dB), dbToPos(smooth100_dB));
                }
            }
        } else {
            gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// -----------------------------------------------------------
// app_main
// -----------------------------------------------------------
extern "C" void app_main(void) {
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "Booting ESP32 A2DP Sink + BLE + DSP...");

    // OTA validation
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA validated");
        }
    }

    // Load settings
    g_settings.load();
    bool bassBoost, channelFlip, bypass;
    int8_t eqBass, eqMid, eqTreble;
    std::string deviceName;
    g_settings.getControl(bassBoost, channelFlip, bypass);
    g_settings.getEQ(eqBass, eqMid, eqTreble);
    g_settings.getDeviceName(deviceName);

    // Initialize DSP
    g_dsp.setSampleRate(APP_I2S_DEFAULT_SAMPLE_RATE);
    g_dsp.setEQ(eqBass, eqMid, eqTreble, APP_I2S_DEFAULT_SAMPLE_RATE);
    g_dsp.setBassBoost(bassBoost);
    g_dsp.setChannelFlip(channelFlip);
    g_dsp.setBypass(bypass);

    // Initialize I2S
    if (g_i2s.init(APP_I2S_DEFAULT_SAMPLE_RATE) != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed");
        return;
    }

    // Initialize audio pipeline
    if (!g_pipeline.init()) {
        ESP_LOGE(TAG, "Audio pipeline init failed");
        return;
    }

    // GPIO init (buttons + LED)
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << APP_BUTTON1_GPIO) | (1ULL << APP_BUTTON2_GPIO);
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    gpio_config_t led = {};
    led.mode = GPIO_MODE_OUTPUT;
    led.pin_bit_mask = (1ULL << APP_BEAT_LED_GPIO);
    gpio_config(&led);
    gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);

    // Initialize BLE
    g_ble.setCallbacks(onBleControl, onBleEq, onBleName, onBleOtaCtrl, onBleOtaData);
    g_ble.init(deviceName.c_str(), APP_FW_VERSION, getControlByte(), eqBass, eqMid, eqTreble);

    // Start A2DP
    g_a2dp.set_output_active(false);
    g_a2dp.set_stream_reader(onStreamData, false);
    g_a2dp.set_codec_config_callback(onCodecConfig);
    g_a2dp.set_auto_reconnect(true);
    g_a2dp.set_task_core(0);
    g_a2dp.set_on_connection_state_changed(onConnectionState);
    g_a2dp.set_on_audio_state_changed(onAudioState);
    g_a2dp.start(deviceName.c_str());
    ESP_LOGI(TAG, "A2DP started as '%s'", deviceName.c_str());

    // Start tasks - audio_tx at very high priority for smooth LDAC playback
    xTaskCreatePinnedToCore(audioTxTask, "audio_tx", 8192, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
    xTaskCreate(buttonsTask, "buttons", 2048, nullptr, 5, nullptr);
    xTaskCreate(beatTask, "beat", 2048, nullptr, 4, nullptr);

    ESP_LOGI(TAG, "System ready");
}
