#include <stdint.h>
#include <string.h>
#include <math.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_ota_ops.h"

#include "idf_update.h"

#include "driver/gpio.h"
#include "driver/i2s.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

#include "BluetoothA2DPSink.h"

// -----------------------------------------------------------
// General config
// -----------------------------------------------------------

static const char *TAG = "A2DP_DSP_BLE";

// I2S
#define I2S_PORT            I2S_NUM_1
#define I2S_BCK_PIN         (27)
#define I2S_LRCK_PIN        (25)
#define I2S_DATA_PIN        (26)
#define I2S_DEFAULT_SR      (44100)

// Buttons and LED
#define BUTTON1_GPIO        (18)
#define BUTTON2_GPIO        (21)
#define BEAT_LED_GPIO       (5)

// DSP constants
static const float BASS_GAIN_BASE   = 1.0f;   // base gain
static const float BASS_GAIN_BOOST  = 1.0f;   // multiplier (shelf provides the boost)
static const float TREBLE_GAIN      = 1.0f;   // no treble adjustment

// Goertzel / beat detect
static const uint16_t GOERTZEL_N = 512;

// Beat detection parameters
static const float BASS_AVG_ALPHA      = 0.01f;
static const float BASS_SMOOTH_ALPHA   = 0.35f;
static const float BASS_MIN_LEVEL      = 0.012f;
static const float BASS_RATIO_THRESH   = 1.6f;
static const uint32_t BEAT_MIN_INTERVAL_MS = 90;
static const uint32_t BEAT_FLASH_DURATION_MS = 60;
static const uint32_t LEVELS_UPDATE_MS = 50;

// NVS keys
static const char *NVS_NAMESPACE   = "audio";
static const char *KEY_DEVNAME     = "devname";
static const char *KEY_CTRL        = "ctrl";
static const char *KEY_EQ_BASS     = "eq_bass";
static const char *KEY_EQ_MID      = "eq_mid";
static const char *KEY_EQ_TREB     = "eq_treb";

// Default name / FW version
static std::string g_device_name = "BDK SPEAKER";
static const char *FW_VER         = "1.2.0";

// -----------------------------------------------------------
// BLE UUIDs (strings, then converted to 128-bit UUIDs)
// -----------------------------------------------------------

#define BLE_SERVICE_UUID_LEVELS   "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHAR_UUID_LEVELS      "12345678-1234-1234-1234-1234567890ac"

#define BLE_SERVICE_UUID_CONTROL  "12345678-1234-1234-1234-1234567890ad"
#define BLE_CHAR_UUID_CONTROL     "12345678-1234-1234-1234-1234567890ae"
#define BLE_CHAR_UUID_EQ          "12345678-1234-1234-1234-1234567890af"
#define BLE_CHAR_UUID_DEVNAME     "12345678-1234-1234-1234-1234567890b0"
#define BLE_CHAR_UUID_FWVER       "12345678-1234-1234-1234-1234567890b3"

#define BLE_CHAR_UUID_OTA_CTRL    "12345678-1234-1234-1234-1234567890b1"
#define BLE_CHAR_UUID_OTA_DATA    "12345678-1234-1234-1234-1234567890b2"

// 128-bit UUID storage in Little-Endian for ESP-IDF
static uint8_t uuid_levels_service[16];
static uint8_t uuid_levels_char[16];

static uint8_t uuid_control_service[16];
static uint8_t uuid_control_char[16];
static uint8_t uuid_eq_char[16];
static uint8_t uuid_name_char[16];
static uint8_t uuid_fw_char[16];
static uint8_t uuid_ota_ctrl_char[16];
static uint8_t uuid_ota_data_char[16];

// -----------------------------------------------------------
// Helper: UUID string -> 128-bit LE
// "12345678-1234-1234-1234-1234567890ab"
// -----------------------------------------------------------
static int hex_nib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0;
}
static void uuid128_from_string(const char *str, uint8_t out[16]) {
    uint8_t tmp[16];
    int idx = 0;
    for (int i = 0; str[i] != '\0' && idx < 16;) {
        if (str[i] == '-') {
            ++i;
            continue;
        }
        int hi = hex_nib(str[i++]);
        int lo = hex_nib(str[i++]);
        tmp[idx++] = (uint8_t)((hi << 4) | lo);
    }
    // BLE expects LE order in esp_bt_uuid_t / adv data
    for (int i = 0; i < 16; ++i) {
        out[i] = tmp[15 - i];
    }
}

static bool uuid_equal_128(const esp_bt_uuid_t &u, const uint8_t raw[16]) {
    if (u.len != ESP_UUID_LEN_128) return false;
    return memcmp(u.uuid.uuid128, raw, 16) == 0;
}

static bool uuid_equal_128_raw(const uint8_t uuid128[16], const uint8_t raw[16]) {
    return memcmp(uuid128, raw, 16) == 0;
}

// -----------------------------------------------------------
// Template clamp
// -----------------------------------------------------------
template<typename T, typename U, typename V>
static inline T clamp_value(U x, V lo, V hi) {
    if (x < lo) return (T)lo;
    if (x > hi) return (T)hi;
    return (T)x;
}

// -----------------------------------------------------------
// A2DP Sink + audio format
// -----------------------------------------------------------

static BluetoothA2DPSink a2dp_sink;

static uint32_t g_sample_rate     = I2S_DEFAULT_SR;
static uint8_t  g_bits_per_sample = 16;   // codec/PCM bits coming from BT stack
static uint8_t  g_channels        = 2;
static bool     g_i2s_initialized = false;

// FIX: Keep I2S ALWAYS 32-bit stereo; only update sample-rate on codec changes.
// This avoids uninstall/reinstall (DMA malloc fail) and prevents slow/noisy playback.
static uint32_t g_i2s_sample_rate = 0;
static volatile bool g_i2s_reconfig = false;
static SemaphoreHandle_t g_i2s_mutex = nullptr;

// (moved earlier)

// Cached EQ enable (faster than checking fabsf per sample)
static bool     g_eq_active = false;

// Disable analysis at high SR to avoid underruns on classic ESP32
static bool     g_enable_analysis = true;

// 1-pole low-pass for bass split
static float lp_alpha = 0.0f;
static float lp_state = 0.0f;

// Output buffer (always 32-bit stereo)
// Output buffer (always 32-bit stereo). Allocate in internal RAM to reduce PSRAM jitter.
static const size_t DSP_OUT_FRAMES = 1024;
static int32_t *dsp_out32 = NULL; // allocated in app_main

// DSP control flags
static bool bassBoostEnabled   = false;
static bool channelFlipEnabled = false;
static bool bypassEnabled      = false;

// -----------------------------------------------------------
// BiQuad filters (for 3-band EQ and optional bass shelf)
// -----------------------------------------------------------

struct Biquad {
    float b0, b1, b2, a1, a2;
    float z1, z2;
    Biquad() : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f), z1(0.0f), z2(0.0f) {}
};

static inline float processSample(Biquad &f, float in) {
    // Transposed Direct Form II (good numeric stability, fast)
    float out = f.b0 * in + f.z1;
    f.z1 = f.b1 * in + f.z2 - f.a1 * out;
    f.z2 = f.b2 * in - f.a2 * out;
    return out;
}

static constexpr float PI_F = 3.14159265f;

static void makeLowShelf(Biquad &f, float fs, float fc, float gainDB) {
    float A = sqrtf(powf(10.0f, gainDB / 20.0f));
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn / 2.0f * sqrtf(2.0f);

    float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;

    float b0 = A * ((A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha);
    float b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cs);
    float b2 = A * ((A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha);
    float a0 = (A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha;
    float a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cs);
    float a2 = (A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha;

    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

static void makeHighShelf(Biquad &f, float fs, float fc, float gainDB) {
    float A = sqrtf(powf(10.0f, gainDB / 20.0f));
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn / 2.0f * sqrtf(2.0f);

    float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;

    float b0 = A * ((A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha);
    float b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cs);
    float b2 = A * ((A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha);
    float a0 = (A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha;
    float a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cs);
    float a2 = (A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha;

    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

static void makePeakingEQ(Biquad &f, float fs, float fc, float Q, float gainDB) {
    float A = sqrtf(powf(10.0f, gainDB / 20.0f));
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn / (2.0f * Q);

    float b0 = 1.0f + alpha * A;
    float b1 = -2.0f * cs;
    float b2 = 1.0f - alpha * A;
    float a0 = 1.0f + alpha / A;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha / A;

    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

// Butterworth 2nd-order Low-Pass Filter (Q = 0.7071 for flat response)
static void makeLowPass(Biquad &f, float fs, float fc) {
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn / (2.0f * 0.7071f); // Butterworth Q

    float b0 = (1.0f - cs) / 2.0f;
    float b1 = 1.0f - cs;
    float b2 = (1.0f - cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;

    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

// Butterworth 2nd-order High-Pass Filter (Q = 0.7071 for flat response)
static void makeHighPass(Biquad &f, float fs, float fc) {
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn / (2.0f * 0.7071f); // Butterworth Q

    float b0 = (1.0f + cs) / 2.0f;
    float b1 = -(1.0f + cs);
    float b2 = (1.0f + cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;

    f.b0 = b0 / a0;
    f.b1 = b1 / a0;
    f.b2 = b2 / a0;
    f.a1 = a1 / a0;
    f.a2 = a2 / a0;
    f.z1 = f.z2 = 0.0f;
}

// EQ filters
static Biquad eqBassL, eqBassR;
static Biquad eqMidL, eqMidR;
static Biquad eqTrebleL, eqTrebleR;

// Optional bass shelf (extra low freq push)
static Biquad bassShelfL;

// Crossover filters for split-ear mode (when not bypassed)
// Left channel: Low-pass at 90Hz, Right channel: High-pass at 500Hz
static Biquad crossoverLPL;  // Low-pass for left channel
static Biquad crossoverLPR;  // Low-pass for right channel (used when flipped)
static Biquad crossoverHPL;  // High-pass for left channel (used when flipped)
static Biquad crossoverHPR;  // High-pass for right channel

static constexpr float CROSSOVER_LP_FREQ = 90.0f;   // Low-pass cutoff (Hz)
static constexpr float CROSSOVER_HP_FREQ = 500.0f;  // High-pass cutoff (Hz)

// EQ gains in dB (controlled via BLE)
static float eqBassDB   = 0.0f;
static float eqMidDB    = 0.0f;
static float eqTrebleDB = 0.0f;

// -----------------------------------------------------------
// Goertzel for 30 / 60 / 100 Hz (beat detector + spectrum)
// -----------------------------------------------------------

struct Goertzel {
    float coeff;
    float s1;
    float s2;
    float freq;

    void init(float f, float fs) {
        freq = f;
        s1 = 0.0f;
        s2 = 0.0f;
        float w = 2.0f * PI_F * f / fs;
        coeff = 2.0f * cosf(w);
    }

    void reset() {
        s1 = 0.0f;
        s2 = 0.0f;
    }

    void feed(float x) {
        float s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float magnitude(float fs) const {
        float w = 2.0f * PI_F * freq / fs;
        float real = s1 - s2 * cosf(w);
        float imag = s2 * sinf(w);
        return sqrtf(real * real + imag * imag);
    }
};

static Goertzel g30, g60, g100;
static uint16_t goertzelCount = 0;

static volatile float g30_dB  = -120.0f;
static volatile float g60_dB  = -120.0f;
static volatile float g100_dB = -120.0f;

static volatile float g30_lin  = 0.0f;
static volatile float g60_lin  = 0.0f;
static volatile float g100_lin = 0.0f;

// smoothed (for BLE levels)
static float smooth30_dB  = -120.0f;
static float smooth60_dB  = -120.0f;
static float smooth100_dB = -120.0f;

static const float FFT_SMOOTH_ALPHA = 0.2f;

// beat detector state
static float bassAvg    = 0.0f;
static float bassSmooth = 0.0f;

static uint32_t lastBeatMs      = 0;
static bool     beatFlashActive = false;
static uint32_t beatFlashOffMs  = 0;

// -----------------------------------------------------------
// BLE / GATT state
// -----------------------------------------------------------

#define PROFILE_APP_ID   0

static esp_gatt_if_t gl_gatts_if = ESP_GATT_IF_NONE;
static uint16_t      gl_conn_id  = 0;
static bool          bleClientConnected = false;

// Service handles
static uint16_t levels_service_handle  = 0;
static uint16_t control_service_handle = 0;

// Characteristic handles
static uint16_t levels_char_handle     = 0;
static uint16_t control_char_handle    = 0;
static uint16_t eq_char_handle         = 0;
static uint16_t name_char_handle       = 0;
static uint16_t fw_char_handle         = 0;
static uint16_t ota_ctrl_char_handle   = 0;
static uint16_t ota_data_char_handle   = 0;

// Attribute values
static uint8_t  control_char_value[1]   = {0};
static uint8_t  eq_char_value[3]        = {0};
static uint8_t  levels_char_value[32]   = {0};
static char     name_char_value[24]     = "BDK SPEAKER";
static char     fw_char_value[16]       = "1.2.0";
static uint8_t  ota_ctrl_char_value[32] = {0};

// BLE advertising
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = nullptr,
    .service_data_len    = 0,
    .p_service_data      = nullptr,

    // advertise CONTROL service UUID
    .service_uuid_len    = 16,
    .p_service_uuid      = uuid_control_service,

    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = false,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = nullptr,
    .service_data_len    = 0,
    .p_service_data      = nullptr,
    .service_uuid_len    = 0,
    .p_service_uuid      = nullptr,
    .flag                = 0,
};

static uint8_t adv_config_done = 0;
static constexpr uint8_t ADV_CONFIG_FLAG = (1 << 0);
static constexpr uint8_t SCAN_RSP_CONFIG_FLAG = (1 << 1);

// -----------------------------------------------------------
// OTA state
// -----------------------------------------------------------

static bool otaInProgress     = false;
static size_t otaExpectedSize = 0;
static size_t otaReceivedBytes= 0;
static bool otaActive         = false;

static IdfUpdate g_update;

static constexpr size_t OTA_PREBEGIN_MAX = 16384;
static uint8_t *otaPrebeginBuf = nullptr;
static size_t otaPrebeginLen = 0;
static bool otaPrebeginOverflow = false;

static bool ota_prebegin_ensure_buf() {
    if (otaPrebeginBuf) return true;
    // Prefer PSRAM to avoid starving internal RAM (I2S DMA needs internal).
    otaPrebeginBuf = (uint8_t *)heap_caps_malloc(OTA_PREBEGIN_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!otaPrebeginBuf) {
        ESP_LOGW(TAG, "OTA: failed to alloc pre-BEGIN buf in SPIRAM, trying internal heap");
        otaPrebeginBuf = (uint8_t *)heap_caps_malloc(OTA_PREBEGIN_MAX, MALLOC_CAP_8BIT);
    }
    if (!otaPrebeginBuf) {
        ESP_LOGE(TAG, "OTA: failed to allocate pre-BEGIN buffer (%u bytes)", (unsigned)OTA_PREBEGIN_MAX);
        return false;
    }
    return true;
}

// -----------------------------------------------------------
// Helpers for millis() using esp_timer
// -----------------------------------------------------------

static uint32_t millis32() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// -----------------------------------------------------------
// NVS helpers (device name, control byte, EQ)
// -----------------------------------------------------------

static uint8_t getControlByte() {
    uint8_t v = 0;
    if (bassBoostEnabled)   v |= 0x01;
    if (channelFlipEnabled) v |= 0x02;
    if (bypassEnabled)      v |= 0x04;
    return v;
}

static void saveControlToNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        uint8_t ctrl = getControlByte();
        nvs_set_u8(h, KEY_CTRL, ctrl);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void saveEQToNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i8(h, KEY_EQ_BASS, (int8_t)eqBassDB);
        nvs_set_i8(h, KEY_EQ_MID,  (int8_t)eqMidDB);
        nvs_set_i8(h, KEY_EQ_TREB, (int8_t)eqTrebleDB);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void saveDeviceNameToNVS() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, KEY_DEVNAME, g_device_name.c_str());
        nvs_commit(h);
        nvs_close(h);
    }
}

static void loadSettingsFromNVS() {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed, using defaults");
        return;
    }

    // device name
    char buf[32];
    size_t len = sizeof(buf);
    err = nvs_get_str(h, KEY_DEVNAME, buf, &len);
    if (err == ESP_OK && len > 0) {
        g_device_name.assign(buf, len - 1); // strip '\0'
    } else {
        g_device_name = "BDK SPEAKER";
    }

    // control byte
    uint8_t ctrl = 0;
    if (nvs_get_u8(h, KEY_CTRL, &ctrl) == ESP_OK) {
        bassBoostEnabled   = (ctrl & 0x01) != 0;
        channelFlipEnabled = (ctrl & 0x02) != 0;
        bypassEnabled      = (ctrl & 0x04) != 0;
    }

    // EQ
    int8_t b = 0, m = 0, t = 0;
    if (nvs_get_i8(h, KEY_EQ_BASS, &b) == ESP_OK) eqBassDB = (float)b;
    if (nvs_get_i8(h, KEY_EQ_MID,  &m) == ESP_OK) eqMidDB  = (float)m;
    if (nvs_get_i8(h, KEY_EQ_TREB, &t) == ESP_OK) eqTrebleDB = (float)t;

    nvs_close(h);

    ESP_LOGI(TAG, "NVS loaded: name='%s', ctrl=0x%02x, EQ(b,m,t)=(%.1f,%.1f,%.1f)",
             g_device_name.c_str(), ctrl, eqBassDB, eqMidDB, eqTrebleDB);
}

// -----------------------------------------------------------
// EQ + filter parameter updates
// -----------------------------------------------------------

static void update_filter_params() {
    const float fc = 300.0f; // crossover freq

    if (g_sample_rate == 0) {
        lp_alpha = 0.0f;
        return;
    }

    float dt = 1.0f / (float)g_sample_rate;
    float rc = 1.0f / (2.0f * PI_F * fc);
    float alpha = dt / (rc + dt);

    if (alpha < 0.001f) alpha = 0.001f;
    if (alpha > 0.999f) alpha = 0.999f;

    lp_alpha = alpha;
    lp_state = 0.0f;

    ESP_LOGI(TAG, "1-pole crossover: fc=%.1f Hz, sr=%u, alpha=%.6f",
             fc, (unsigned)g_sample_rate, (double)lp_alpha);
}

static void updateEQFilters() {
    if (g_sample_rate == 0) return;
    float fs = (float)g_sample_rate;

    makeLowShelf(eqBassL, fs, 150.0f, eqBassDB);
    makeLowShelf(eqBassR, fs, 150.0f, eqBassDB);
    makePeakingEQ(eqMidL, fs, 1000.0f, 1.0f, eqMidDB);
    makePeakingEQ(eqMidR, fs, 1000.0f, 1.0f, eqMidDB);
    makeHighShelf(eqTrebleL, fs, 6000.0f, eqTrebleDB);
    makeHighShelf(eqTrebleR, fs, 6000.0f, eqTrebleDB);

    // Bass boost shelf (+2 dB at 200 Hz)
    makeLowShelf(bassShelfL, fs, 200.0f, 2.0f);

    // Initialize crossover filters for split-ear mode
    makeLowPass(crossoverLPL, fs, CROSSOVER_LP_FREQ);
    makeLowPass(crossoverLPR, fs, CROSSOVER_LP_FREQ);
    makeHighPass(crossoverHPL, fs, CROSSOVER_HP_FREQ);
    makeHighPass(crossoverHPR, fs, CROSSOVER_HP_FREQ);
    ESP_LOGI(TAG, "Crossover filters: LP=%.0f Hz, HP=%.0f Hz", CROSSOVER_LP_FREQ, CROSSOVER_HP_FREQ);

    // Cache active state
    g_eq_active = (fabsf(eqBassDB) >= 0.1f) ||
                  (fabsf(eqMidDB)  >= 0.1f) ||
                  (fabsf(eqTrebleDB) >= 0.1f);

    ESP_LOGI(TAG, "EQ updated: bass=%.1f dB, mid=%.1f dB, treble=%.1f dB (active=%d)",
             eqBassDB, eqMidDB, eqTrebleDB, (int)g_eq_active);
}

static inline void applyEQ(float &L, float &R) {
    if (!g_eq_active) return;

    float l = L;
    float r = R;

    l = processSample(eqBassL, l);
    r = processSample(eqBassR, r);

    l = processSample(eqMidL, l);
    r = processSample(eqMidR, r);

    l = processSample(eqTrebleL, l);
    r = processSample(eqTrebleR, r);

    L = l;
    R = r;
}

// -----------------------------------------------------------
// Goertzel processing per sample (mono analysis)
// -----------------------------------------------------------

static void goertzelInitAll() {
    if (g_sample_rate == 0) return;
    float fs = (float)g_sample_rate;
    g30.init(30.0f, fs);
    g60.init(60.0f, fs);
    g100.init(100.0f, fs);
    goertzelCount = 0;
}

static void goertzelProcessSample(float x) {
    g30.feed(x);
    g60.feed(x);
    g100.feed(x);

    goertzelCount++;
    if (goertzelCount >= GOERTZEL_N) {
        float fs = (float)g_sample_rate;
        if (fs <= 0.0f) fs = (float)I2S_DEFAULT_SR;

        float m30 = g30.magnitude(fs);
        float m60 = g60.magnitude(fs);
        float m100 = g100.magnitude(fs);

        // Normalize by N/2 for float audio in -1.0 to 1.0 range
        // Full-scale sine at target freq yields magnitude ~N/2
        const float normFactor = (float)(GOERTZEL_N / 2);
        float norm30 = m30 / normFactor;
        float norm60 = m60 / normFactor;
        float norm100 = m100 / normFactor;

        if (norm30 < 1e-9f) norm30 = 1e-9f;
        if (norm60 < 1e-9f) norm60 = 1e-9f;
        if (norm100 < 1e-9f) norm100 = 1e-9f;

        float dB30 = 20.0f * log10f(norm30);
        float dB60 = 20.0f * log10f(norm60);
        float dB100 = 20.0f * log10f(norm100);

        if (dB30 < -120.0f || !isfinite(dB30)) dB30 = -120.0f;
        if (dB60 < -120.0f || !isfinite(dB60)) dB60 = -120.0f;
        if (dB100 < -120.0f || !isfinite(dB100)) dB100 = -120.0f;

        g30_dB  = dB30;
        g60_dB  = dB60;
        g100_dB = dB100;

        g30_lin  = norm30;
        g60_lin  = norm60;
        g100_lin = norm100;

        g30.reset();
        g60.reset();
        g100.reset();
        goertzelCount = 0;
    }
}

// -----------------------------------------------------------
// I2S configuration (init once, then update clock only)
// FIX: Avoid uninstall/reinstall on codec changes.
// -----------------------------------------------------------

static esp_err_t configure_i2s_init_once(uint32_t sample_rate) {
    if (g_i2s_initialized) return ESP_OK;

    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.sample_rate = sample_rate;

    // ALWAYS 32-bit container output (stable across 16/24/32-bit codec PCM)
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;

    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB);
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;

    // Increase DMA buffering to reduce underruns/ choppy audio (trade latency)
    // NOTE: these buffers allocate DMA-capable INTERNAL RAM. If internal RAM is tight,
    // we fall back to smaller buffers below.
    i2s_config.dma_buf_count = 12;
    i2s_config.dma_buf_len   = 512;

    // Keep APLL off for stability (you can enable if your hardware needs it)
    i2s_config.use_apll = false;

    i2s_config.tx_desc_auto_clear = true;
    i2s_config.fixed_mclk = 0;

    auto install_with = [&](int dma_count, int dma_len) -> esp_err_t {
        i2s_config.dma_buf_count = dma_count;
        i2s_config.dma_buf_len   = dma_len;
        esp_err_t e = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "i2s_driver_install failed (dma_count=%d dma_len=%d): %s", dma_count, dma_len, esp_err_to_name(e));
        }
        return e;
    };

    esp_err_t err = install_with(12, 512);
    if (err == ESP_ERR_NO_MEM) {
        // Try smaller buffers to keep system alive when internal memory is constrained.
        err = install_with(8, 256);
    }
    if (err != ESP_OK) {
        return err;
    }

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = I2S_BCK_PIN;
    pins.ws_io_num    = I2S_LRCK_PIN;
    pins.data_out_num = I2S_DATA_PIN;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;

    // Validate pins before calling into driver (helps avoid invalid pointer deref inside
    // lower-level matrix code if something is misconfigured or driver state is null).
    auto valid_pin = [](int pin) -> bool {
        if (pin == I2S_PIN_NO_CHANGE) return true;
        if (pin >= 0 && pin <= 39) return true;
        return false;
    };
    if (!valid_pin(pins.bck_io_num) || !valid_pin(pins.ws_io_num) || !valid_pin(pins.data_out_num) || !valid_pin(pins.data_in_num)) {
        ESP_LOGE(TAG, "Invalid I2S pin configuration: bck=%d ws=%d dout=%d din=%d",
                 pins.bck_io_num, pins.ws_io_num, pins.data_out_num, pins.data_in_num);
        i2s_driver_uninstall(I2S_PORT);
        return ESP_ERR_INVALID_ARG;
    }

    err = i2s_set_pin(I2S_PORT, &pins);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_PORT);
        return err;
    }

    // Explicit clock setup: 32-bit stereo
    err = i2s_set_clk(I2S_PORT, sample_rate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_clk failed: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_PORT);
        return err;
    }

    g_i2s_initialized = true;
    g_i2s_sample_rate = sample_rate;

    ESP_LOGI(TAG, "I2S configured (init): sr=%u, 32-bit stereo", (unsigned)sample_rate);
    return ESP_OK;
}

static void i2s_update_clock(uint32_t sample_rate) {
    if (!g_i2s_initialized) return;

    if (sample_rate == 0) sample_rate = I2S_DEFAULT_SR;
    if (g_i2s_sample_rate == sample_rate) return;

    g_i2s_reconfig = true;

    i2s_stop(I2S_PORT);
    i2s_zero_dma_buffer(I2S_PORT);

    esp_err_t err = i2s_set_clk(I2S_PORT, sample_rate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_clk failed: %s", esp_err_to_name(err));
    } else {
        g_i2s_sample_rate = sample_rate;
        ESP_LOGI(TAG, "I2S clock updated: sr=%u (32-bit stereo)", (unsigned)sample_rate);
    }

    i2s_start(I2S_PORT);
    g_i2s_reconfig = false;
}

// -----------------------------------------------------------
// A2DP codec config callback (rate/bits/channels)
// Called when codec configuration changes (e.g., switching between LDAC/SBC/AAC)
// -----------------------------------------------------------

static void codec_config_cb(uint32_t rate, uint8_t bps, uint8_t channels) {
    ESP_LOGI(TAG, ">>> CODEC CONFIG: rate=%u, bits=%u, ch=%u (prev sr=%u)",
             (unsigned)rate, (unsigned)bps, (unsigned)channels, (unsigned)g_sample_rate);

    uint32_t new_rate = (rate != 0) ? rate : I2S_DEFAULT_SR;
    uint8_t new_bps = bps;
    uint8_t new_ch = channels ? channels : 2;

    // Check if this is actually a change
    bool rate_changed = (new_rate != g_sample_rate);
    bool format_changed = (new_bps != g_bits_per_sample) || (new_ch != g_channels);
    
    g_sample_rate     = new_rate;
    g_bits_per_sample = new_bps;
    g_channels        = new_ch;

    // Enable analysis for all sample rates (level meters needed)
    g_enable_analysis = true;

    // Guard: if heap is low, skip reconfig to avoid driver/dma allocation failure
    size_t free_heap = esp_get_free_heap_size();
    const size_t kMinHeapForI2S = 32 * 1024; // 32KB safe margin
    if (free_heap < kMinHeapForI2S) {
        ESP_LOGW(TAG, "Low heap (%u bytes), skipping I2S reconfiguration to avoid crash", (unsigned)free_heap);
        update_filter_params();
        updateEQFilters();
        goertzelInitAll();
        return;
    }

    if (g_i2s_mutex) xSemaphoreTake(g_i2s_mutex, portMAX_DELAY);

    if (!g_i2s_initialized) {
        esp_err_t e = configure_i2s_init_once(g_sample_rate);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "configure_i2s_init_once failed: %s", esp_err_to_name(e));
        }
    } else if (rate_changed) {
        // Force I2S clock update even if sample rate looks the same
        // This helps with codec switching where the rate might be reported
        // in a different order
        ESP_LOGI(TAG, "Sample rate changed: %u -> %u, updating I2S", 
                 (unsigned)g_i2s_sample_rate, (unsigned)g_sample_rate);
        
        g_i2s_reconfig = true;
        i2s_stop(I2S_PORT);
        i2s_zero_dma_buffer(I2S_PORT);
        
        esp_err_t err = i2s_set_clk(I2S_PORT, g_sample_rate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_set_clk failed: %s", esp_err_to_name(err));
        } else {
            g_i2s_sample_rate = g_sample_rate;
            ESP_LOGI(TAG, "I2S clock updated: sr=%u (32-bit stereo)", (unsigned)g_sample_rate);
        }
        
        i2s_start(I2S_PORT);
        g_i2s_reconfig = false;
    }

    if (g_i2s_mutex) xSemaphoreGive(g_i2s_mutex);

    // Reset DSP filter state for new sample rate
    lp_state = 0.0f;
    update_filter_params();
    updateEQFilters();
    goertzelInitAll();
    
    ESP_LOGI(TAG, "<<< CODEC CONFIG complete: I2S sr=%u, analysis=%s",
             (unsigned)g_i2s_sample_rate, g_enable_analysis ? "ON" : "OFF");
}

// -----------------------------------------------------------
// BLE helpers to update characteristics + notify
// No CCCD checks - always send notifications when connected
// -----------------------------------------------------------

static void ble_update_control_char() {
    if (control_char_handle == 0) {
        ESP_LOGW(TAG, "control_char_handle is 0, skipping");
        return;
    }
    uint8_t v = getControlByte();
    control_char_value[0] = v;
    esp_err_t err = esp_ble_gatts_set_attr_value(control_char_handle, 1, control_char_value);
    ESP_LOGI(TAG, "Control attr set: 0x%02x, err=%d", v, err);
    
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, control_char_handle, 1, control_char_value, false);
        ESP_LOGI(TAG, "Control notify sent: 0x%02x, err=%d", v, err);
    }
}

static void ble_update_eq_char() {
    if (eq_char_handle == 0) {
        ESP_LOGW(TAG, "eq_char_handle is 0, skipping");
        return;
    }
    int8_t vals[3] = {
        (int8_t)eqBassDB,
        (int8_t)eqMidDB,
        (int8_t)eqTrebleDB
    };
    memcpy(eq_char_value, vals, 3);
    esp_err_t err = esp_ble_gatts_set_attr_value(eq_char_handle, 3, eq_char_value);
    ESP_LOGI(TAG, "EQ attr set: bass=%d mid=%d treb=%d, err=%d", vals[0], vals[1], vals[2], err);
    
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, eq_char_handle, 3, eq_char_value, false);
        ESP_LOGI(TAG, "EQ notify sent, err=%d", err);
    }
}

static void ble_update_name_char() {
    if (name_char_handle == 0) {
        ESP_LOGW(TAG, "name_char_handle is 0, skipping");
        return;
    }
    size_t len = g_device_name.size();
    if (len > sizeof(name_char_value) - 1) len = sizeof(name_char_value) - 1;
    memcpy(name_char_value, g_device_name.c_str(), len);
    name_char_value[len] = '\0';
    esp_err_t err = esp_ble_gatts_set_attr_value(name_char_handle, len, (uint8_t *)name_char_value);
    ESP_LOGI(TAG, "Name attr set: '%s', err=%d", name_char_value, err);
    
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, name_char_handle, len, (uint8_t *)name_char_value, false);
        ESP_LOGI(TAG, "Name notify sent, err=%d", err);
    }
}

static void ble_update_fw_char() {
    if (fw_char_handle == 0) {
        ESP_LOGW(TAG, "fw_char_handle is 0, skipping");
        return;
    }
    size_t len = strlen(FW_VER);
    memcpy(fw_char_value, FW_VER, len);
    fw_char_value[len] = '\0';
    esp_err_t err = esp_ble_gatts_set_attr_value(fw_char_handle, len, (uint8_t *)fw_char_value);
    ESP_LOGI(TAG, "FW attr set: '%s', err=%d", fw_char_value, err);
    
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        err = esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, fw_char_handle, len, (uint8_t *)fw_char_value, false);
        ESP_LOGI(TAG, "FW notify sent, err=%d", err);
    }
}

static void ble_update_levels_char(int l30, int l60, int l100) {
    if (levels_char_handle == 0) {
        ESP_LOGW(TAG, "levels_char_handle is 0, skipping");
        return;
    }
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%d,%d,%d", l30, l60, l100);
    if (len < 0) return;
    memcpy(levels_char_value, buf, len);
    esp_ble_gatts_set_attr_value(levels_char_handle, len, levels_char_value);
    
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        esp_err_t err = esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, levels_char_handle, len, levels_char_value, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Levels notify failed: err=%d", err);
        }
    }
}

static void ble_notify_ota_ctrl(const char *msg) {
    if (ota_ctrl_char_handle == 0) return;
    size_t len = strlen(msg);
    if (len > sizeof(ota_ctrl_char_value)) len = sizeof(ota_ctrl_char_value);
    memcpy(ota_ctrl_char_value, msg, len);
    esp_ble_gatts_set_attr_value(ota_ctrl_char_handle, len, ota_ctrl_char_value);
    if (bleClientConnected && gl_gatts_if != ESP_GATT_IF_NONE) {
        esp_ble_gatts_send_indicate(gl_gatts_if, gl_conn_id, ota_ctrl_char_handle, len, ota_ctrl_char_value, false);
    }
}


// -----------------------------------------------------------
// Control / EQ / Name from BLE writes
// -----------------------------------------------------------

static void applyControlByte(uint8_t v) {
    bassBoostEnabled   = (v & 0x01) != 0;
    channelFlipEnabled = (v & 0x02) != 0;
    bypassEnabled      = (v & 0x04) != 0;

    ESP_LOGI(TAG, "BLE control: BassBoost=%d, Flip=%d, Bypass=%d",
             bassBoostEnabled, channelFlipEnabled, bypassEnabled);

    ble_update_control_char();
    saveControlToNVS();
}

static void applyEqBytes(const uint8_t *data, size_t len) {
    if (len < 3) return;
    int8_t b = (int8_t)data[0];
    int8_t m = (int8_t)data[1];
    int8_t t = (int8_t)data[2];

    eqBassDB   = (float)b;
    eqMidDB    = (float)m;
    eqTrebleDB = (float)t;

    ESP_LOGI(TAG, "BLE EQ: Bass=%.1f, Mid=%.1f, Treble=%.1f",
             eqBassDB, eqMidDB, eqTrebleDB);

    updateEQFilters();
    ble_update_eq_char();
    saveEQToNVS();
}

static void applyDeviceNameWrite(const uint8_t *data, uint16_t len) {
    if (len == 0) return;
    std::string val((const char *)data, len);
    // trim spaces
    size_t start = val.find_first_not_of(" \t\r\n");
    size_t end   = val.find_last_not_of(" \t\r\n");
    if (start == std::string::npos) return;
    val = val.substr(start, end - start + 1);
    if (val.empty()) return;
    if (val.size() > 20) val.resize(20);

    g_device_name = val;
    saveDeviceNameToNVS();

    ESP_LOGI(TAG, "BLE name updated -> '%s'", g_device_name.c_str());

    // apply to BLE GAP
    esp_ble_gap_set_device_name(g_device_name.c_str());
    ble_update_name_char();

    ESP_LOGW(TAG, "After changing name, reboot is recommended to fully restart A2DP");
}

// -----------------------------------------------------------
// OTA via BLE (BEGIN / DATA / END / ABORT)
// -----------------------------------------------------------

static void ota_reset_state() {
    otaInProgress      = false;
    otaExpectedSize    = 0;
    otaReceivedBytes   = 0;
    otaActive          = false;
    if (g_update.isRunning()) {
        g_update.abort();
    }
    if (otaPrebeginBuf) {
        heap_caps_free(otaPrebeginBuf);
        otaPrebeginBuf = nullptr;
    }
    otaPrebeginLen = 0;
    otaPrebeginOverflow = false;
}

static void handleOtaCtrlWrite(const uint8_t *data, uint16_t len) {
    if (!data || len == 0) return;

    std::string cmd((const char *)data, len);
    // trim
    size_t s = cmd.find_first_not_of(" \r\n\t");
    size_t e = cmd.find_last_not_of(" \r\n\t");
    if (s == std::string::npos) return;
    cmd = cmd.substr(s, e - s + 1);

    ESP_LOGI(TAG, "OTA CTRL: '%s'", cmd.c_str());

    if (cmd.rfind("BEGIN:", 0) == 0) {
        std::string sizeStr = cmd.substr(6);
        sizeStr.erase(0, sizeStr.find_first_not_of(" \t"));
        otaExpectedSize = (size_t)atoi(sizeStr.c_str());
        ESP_LOGI(TAG, "OTA BEGIN, size=%u", (unsigned)otaExpectedSize);

        if (otaExpectedSize == 0) {
            ESP_LOGE(TAG, "OTA: invalid size 0");
            ota_reset_state();
            return;
        }

        // stop audio playback while OTA
        a2dp_sink.stop();

        if (!g_update.begin(otaExpectedSize)) {
            ESP_LOGE(TAG, "OTA: begin failed: %s (esp_err=%s)", g_update.errorString(), esp_err_to_name(g_update.lastEspErr()));
            ble_notify_ota_ctrl("BEGIN_ERR");
            ota_reset_state();
            return;
        }

        otaInProgress    = true;
        otaReceivedBytes = 0;
        otaActive        = true;

        if (otaPrebeginOverflow) {
            ESP_LOGW(TAG, "OTA: pre-BEGIN buffer overflowed; some bytes were dropped");
        }
        if (otaPrebeginLen > 0) {
            ESP_LOGW(TAG, "OTA: flushing %u pre-BEGIN bytes", (unsigned)otaPrebeginLen);
            if (!otaPrebeginBuf) {
                ESP_LOGE(TAG, "OTA: pre-BEGIN buffer missing");
                ble_notify_ota_ctrl("WRITE_ERR");
                ota_reset_state();
                return;
            }
            size_t w = g_update.write(otaPrebeginBuf, otaPrebeginLen);
            if (w != otaPrebeginLen) {
                ESP_LOGE(TAG, "OTA: failed flushing pre-BEGIN bytes (wrote %u/%u): %s", (unsigned)w, (unsigned)otaPrebeginLen, g_update.errorString());
                ble_notify_ota_ctrl("WRITE_ERR");
                ota_reset_state();
                return;
            }
            otaReceivedBytes += w;
            otaPrebeginLen = 0;
            otaPrebeginOverflow = false;
            heap_caps_free(otaPrebeginBuf);
            otaPrebeginBuf = nullptr;
        }

        ble_notify_ota_ctrl("BEGIN_OK");
        return;
    }

    if (cmd == "END") {
        ESP_LOGI(TAG, "OTA END received");
        if (!otaInProgress) {
            ESP_LOGW(TAG, "OTA END but no OTA in progress");
            otaActive = false;
            return;
        }

        if (!g_update.end(false)) {
            ESP_LOGE(TAG, "OTA: end failed: %s (esp_err=%s)", g_update.errorString(), esp_err_to_name(g_update.lastEspErr()));
            ble_notify_ota_ctrl("END_ERR");
            ota_reset_state();
            return;
        }

        ESP_LOGI(TAG, "OTA SUCCESS, restarting...");
        ble_notify_ota_ctrl("OK_REBOOT");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }

    if (cmd == "ABORT") {
        ESP_LOGW(TAG, "OTA ABORT received");
        if (otaInProgress) {
            g_update.abort();
        }
        ota_reset_state();
        ble_notify_ota_ctrl("ABORTED");
        return;
    }

    ESP_LOGW(TAG, "OTA CTRL: unknown command '%s'", cmd.c_str());
}

static void handleOtaDataWrite(const uint8_t *data, uint16_t len) {
    if (!data || len == 0) return;

    // Allow early DATA before BEGIN by buffering a small window.
    if (!otaInProgress || !otaActive) {
        if (!ota_prebegin_ensure_buf()) {
            otaPrebeginOverflow = true;
            return;
        }
        if ((otaPrebeginLen + len) <= OTA_PREBEGIN_MAX) {
            memcpy(&otaPrebeginBuf[otaPrebeginLen], data, len);
            otaPrebeginLen += len;
        } else {
            otaPrebeginOverflow = true;
        }
        return;
    }

    size_t w = g_update.write(data, len);
    if (w != len) {
        ESP_LOGE(TAG, "OTA write error (wrote %u/%u): %s (esp_err=%s)", (unsigned)w, (unsigned)len, g_update.errorString(), esp_err_to_name(g_update.lastEspErr()));
        ble_notify_ota_ctrl("WRITE_ERR");
        ota_reset_state();
        return;
    }

    otaReceivedBytes += w;

    static size_t lastNotified = 0;
    if ((otaReceivedBytes - lastNotified) >= 4096 || otaReceivedBytes == otaExpectedSize) {
        lastNotified = otaReceivedBytes;
        char buf[32];
        snprintf(buf, sizeof(buf), "PROG:%u/%u",
                 (unsigned)otaReceivedBytes, (unsigned)otaExpectedSize);
        ble_notify_ota_ctrl(buf);
    }

    ESP_LOGI(TAG, "OTA progress: %u / %u",
             (unsigned)otaReceivedBytes, (unsigned)otaExpectedSize);
}

// -----------------------------------------------------------
// BLE GAP event handler
// -----------------------------------------------------------

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (uint8_t)~ADV_CONFIG_FLAG;
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (uint8_t)~SCAN_RSP_CONFIG_FLAG;
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed, status=0x%x", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Advertising started");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising stopped");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Conn params updated");
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------
// BLE GATTS callback
// -----------------------------------------------------------

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, app_id=%d", param->reg.app_id);
        gl_gatts_if = gatts_if;

        // Prepare advertising data
        adv_data.set_scan_rsp = false;
        // Keep ADV packet small so the 128-bit CONTROL UUID is not dropped.
        // Put the name into scan response instead.
        adv_data.include_name = false;
        adv_data.include_txpower = true;
        adv_data.min_interval = 0x0006;
        adv_data.max_interval = 0x0010;
        adv_data.appearance = 0x00;
        adv_data.manufacturer_len = 0;
        adv_data.p_manufacturer_data = NULL;
        adv_data.service_data_len = 0;
        adv_data.p_service_data = NULL;
        adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

        // advertise CONTROL service UUID
        adv_data.service_uuid_len = 16;
        adv_data.p_service_uuid   = uuid_control_service;

        esp_ble_gap_set_device_name(g_device_name.c_str());
        adv_config_done = (uint8_t)(ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp_data);

        // Create LEVELS service
        esp_gatt_srvc_id_t svc_id_levels = {};
        svc_id_levels.is_primary = true;
        svc_id_levels.id.inst_id = 0x00;
        svc_id_levels.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc_id_levels.id.uuid.uuid.uuid128, uuid_levels_service, 16);
        esp_ble_gatts_create_service(gatts_if, &svc_id_levels, 4);

        // Create CONTROL service
        esp_gatt_srvc_id_t svc_id_ctrl = {};
        svc_id_ctrl.is_primary = true;
        svc_id_ctrl.id.inst_id = 0x01;
        svc_id_ctrl.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc_id_ctrl.id.uuid.uuid.uuid128, uuid_control_service, 16);
        esp_ble_gatts_create_service(gatts_if, &svc_id_ctrl, 20);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "ESP_GATTS_CREATE_EVT, service_handle=%d, inst_id=%d",
                 param->create.service_handle, param->create.service_id.id.inst_id);

        // Identify which service got created by UUID (more robust than inst_id).
        bool is_levels = false;
        bool is_control = false;
        if (param->create.service_id.id.uuid.len == ESP_UUID_LEN_128) {
            is_levels = uuid_equal_128_raw(param->create.service_id.id.uuid.uuid.uuid128, uuid_levels_service);
            is_control = uuid_equal_128_raw(param->create.service_id.id.uuid.uuid.uuid128, uuid_control_service);
        }

        if (is_levels) {
            levels_service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(levels_service_handle);

            // Response by app for descriptors
            static esp_attr_control_t rsp_by_app_levels = { .auto_rsp = ESP_GATT_RSP_BY_APP };

            // Add LEVELS characteristic
            esp_bt_uuid_t char_uuid = {};
            char_uuid.len = ESP_UUID_LEN_128;
            memcpy(char_uuid.uuid.uuid128, uuid_levels_char, 16);

            esp_attr_value_t char_val = {};
            char_val.attr_max_len = sizeof(levels_char_value);
            char_val.attr_len     = 0;
            char_val.attr_value   = levels_char_value;

            esp_err_t err = esp_ble_gatts_add_char(levels_service_handle,
                                   &char_uuid,
                                   ESP_GATT_PERM_READ,
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &char_val,
                                   &rsp_by_app_levels);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add LEVELS char: %s", esp_err_to_name(err));
            }
        } else if (is_control) {
            control_service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(control_service_handle);

            // Response by app - required to receive WRITE_EVT callbacks
            static esp_attr_control_t rsp_by_app = { .auto_rsp = ESP_GATT_RSP_BY_APP };

            // CONTROL characteristic (flags)
            esp_bt_uuid_t uuid_ctrl = {};
            uuid_ctrl.len = ESP_UUID_LEN_128;
            memcpy(uuid_ctrl.uuid.uuid128, uuid_control_char, 16);
            esp_attr_value_t ctrl_val = {};
            ctrl_val.attr_max_len = sizeof(control_char_value);
            ctrl_val.attr_len     = 1;
            ctrl_val.attr_value   = control_char_value;
            esp_err_t err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_ctrl,
                                   (esp_gatt_perm_t)(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ |
                                                          ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &ctrl_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add CONTROL char: %s", esp_err_to_name(err));
            }

            // EQ characteristic
            esp_bt_uuid_t uuid_eq = {};
            uuid_eq.len = ESP_UUID_LEN_128;
            memcpy(uuid_eq.uuid.uuid128, uuid_eq_char, 16);
            esp_attr_value_t eq_val = {};
            eq_val.attr_max_len = sizeof(eq_char_value);
            eq_val.attr_len     = 3;
            eq_val.attr_value   = eq_char_value;
            err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_eq,
                                   (esp_gatt_perm_t)(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ |
                                                          ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &eq_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add EQ char: %s", esp_err_to_name(err));
            }

            // Name characteristic
            esp_bt_uuid_t uuid_name = {};
            uuid_name.len = ESP_UUID_LEN_128;
            memcpy(uuid_name.uuid.uuid128, uuid_name_char, 16);
            esp_attr_value_t name_val = {};
            name_val.attr_max_len = sizeof(name_char_value);
            name_val.attr_len     = strlen(name_char_value);
            name_val.attr_value   = (uint8_t *)name_char_value;
            err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_name,
                                   (esp_gatt_perm_t)(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ |
                                                          ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &name_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add NAME char: %s", esp_err_to_name(err));
            }

            // FW version characteristic (read + notify)
            esp_bt_uuid_t uuid_fw = {};
            uuid_fw.len = ESP_UUID_LEN_128;
            memcpy(uuid_fw.uuid.uuid128, uuid_fw_char, 16);
            esp_attr_value_t fw_val = {};
            fw_val.attr_max_len = sizeof(fw_char_value);
            fw_val.attr_len     = strlen(fw_char_value);
            fw_val.attr_value   = (uint8_t *)fw_char_value;
            err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_fw,
                                   ESP_GATT_PERM_READ,
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ |
                                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &fw_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add FW char: %s", esp_err_to_name(err));
            }

            // OTA_CTRL characteristic
            esp_bt_uuid_t uuid_ota_ctrl = {};
            uuid_ota_ctrl.len = ESP_UUID_LEN_128;
            memcpy(uuid_ota_ctrl.uuid.uuid128, uuid_ota_ctrl_char, 16);
            esp_attr_value_t ota_ctrl_val = {};
            ota_ctrl_val.attr_max_len = sizeof(ota_ctrl_char_value);
            ota_ctrl_val.attr_len     = 0;
            ota_ctrl_val.attr_value   = ota_ctrl_char_value;
            err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_ota_ctrl,
                                   ESP_GATT_PERM_WRITE,
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                          ESP_GATT_CHAR_PROP_BIT_NOTIFY),
                                   &ota_ctrl_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add OTA_CTRL char: %s", esp_err_to_name(err));
            }

            // OTA_DATA characteristic (write only)
            esp_bt_uuid_t uuid_ota_data = {};
            uuid_ota_data.len = ESP_UUID_LEN_128;
            memcpy(uuid_ota_data.uuid.uuid128, uuid_ota_data_char, 16);
            esp_attr_value_t ota_data_val = {};
            uint8_t dummy = 0;
            ota_data_val.attr_max_len = 512;
            ota_data_val.attr_len     = 1;
            ota_data_val.attr_value   = &dummy;
            err = esp_ble_gatts_add_char(control_service_handle,
                                   &uuid_ota_data,
                                   ESP_GATT_PERM_WRITE,
                                   (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                          ESP_GATT_CHAR_PROP_BIT_WRITE_NR),
                                   &ota_data_val,
                                   &rsp_by_app);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add OTA_DATA char: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "CREATE_EVT for unknown service (uuid_len=%u)", (unsigned)param->create.service_id.id.uuid.len);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        ESP_LOGI(TAG, "ESP_GATTS_ADD_CHAR_EVT, attr_handle=%d", param->add_char.attr_handle);

        // CCCD UUID for adding descriptors (Android app needs these to enable notifications)
        static const uint8_t cccd_uuid_raw[2] = {0x02, 0x29}; // 0x2902 little-endian
        esp_bt_uuid_t cccd_uuid = {};
        cccd_uuid.len = ESP_UUID_LEN_16;
        cccd_uuid.uuid.uuid16 = 0x2902;

        // Store characteristic handles and add CCCD where needed
        bool needs_cccd = false;
        
        if (uuid_equal_128(param->add_char.char_uuid, uuid_levels_char)) {
            levels_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "LEVELS char handle=%u", (unsigned)levels_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_control_char)) {
            control_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "CONTROL char handle=%u", (unsigned)control_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_eq_char)) {
            eq_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "EQ char handle=%u", (unsigned)eq_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_name_char)) {
            name_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "NAME char handle=%u", (unsigned)name_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_fw_char)) {
            fw_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "FW char handle=%u", (unsigned)fw_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_ota_ctrl_char)) {
            ota_ctrl_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "OTA_CTRL char handle=%u", (unsigned)ota_ctrl_char_handle);
            needs_cccd = true;
        } else if (uuid_equal_128(param->add_char.char_uuid, uuid_ota_data_char)) {
            ota_data_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "OTA_DATA char handle=%u", (unsigned)ota_data_char_handle);
            // OTA_DATA doesn't need CCCD (write-only)
        }

        // Add CCCD descriptor for characteristics that support notifications
        if (needs_cccd) {
            // Use the service handle from the event, which is correct for that characteristic
            uint16_t service_handle = param->add_char.service_handle;
            ESP_LOGI(TAG, "Adding CCCD to service_handle=%u for char_handle=%u", 
                     (unsigned)service_handle, (unsigned)param->add_char.attr_handle);
            
            // CCCD needs RSP_BY_APP to receive write events
            static esp_attr_control_t cccd_rsp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
            
            esp_err_t err = esp_ble_gatts_add_char_descr(
                service_handle,
                &cccd_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, &cccd_rsp);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add CCCD: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Added CCCD for char handle=%u", (unsigned)param->add_char.attr_handle);
            }
        }

        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        // CCCD descriptor added - just log it
        ESP_LOGI(TAG, "CCCD descriptor added, handle=%u (status=%d)", 
                 (unsigned)param->add_char_descr.attr_handle, param->add_char_descr.status);
        break;
    }

    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(TAG, "BLE client connected, conn_id=%d", param->connect.conn_id);
        gl_conn_id = param->connect.conn_id;
        bleClientConnected = true;

        // Debug: print all handle mappings
        ESP_LOGI(TAG, "=== Handle mappings ===");
        ESP_LOGI(TAG, "levels_char=%u", (unsigned)levels_char_handle);
        ESP_LOGI(TAG, "control_char=%u", (unsigned)control_char_handle);
        ESP_LOGI(TAG, "eq_char=%u", (unsigned)eq_char_handle);
        ESP_LOGI(TAG, "name_char=%u", (unsigned)name_char_handle);
        ESP_LOGI(TAG, "fw_char=%u", (unsigned)fw_char_handle);
        ESP_LOGI(TAG, "ota_ctrl_char=%u", (unsigned)ota_ctrl_char_handle);
        ESP_LOGI(TAG, "ota_data_char=%u", (unsigned)ota_data_char_handle);
        ESP_LOGI(TAG, "=======================");

        // Small delay to let the connection stabilize before sending notifications
        vTaskDelay(pdMS_TO_TICKS(100));

        // Send all data to client upon connection
        // Name, FW Version, Control (Bass Boost, Channel Flip, Bypass), EQ, and initial levels
        ESP_LOGI(TAG, "Sending initial data to client upon connection...");
        
        // Send FW Version first
        ble_update_fw_char();
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Send Control (Bass Boost, Channel Flip, Bypass)
        ble_update_control_char();
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Send EQ settings
        ble_update_eq_char();
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Send Device Name
        ble_update_name_char();
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Send initial level meter data
        auto dbToPos = [](float dB) -> int {
            int v = (int)roundf(dB + 120.0f);
            if (v < 0) v = 0;
            if (v > 120) v = 120;
            return v;
        };
        ble_update_levels_char(dbToPos(smooth30_dB), dbToPos(smooth60_dB), dbToPos(smooth100_dB));
        
        ESP_LOGI(TAG, "Initial data sent to BLE client");
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(TAG, "BLE client disconnected");
        bleClientConnected = false;
        gl_conn_id = 0;

        esp_ble_gap_start_advertising(&adv_params);
        break;
    }

    case ESP_GATTS_READ_EVT: {
        uint16_t handle = param->read.handle;
        ESP_LOGI(TAG, "READ_EVT handle=%d conn_id=%d trans_id=%d", handle, param->read.conn_id, param->read.trans_id);
        
        // Prepare response
        esp_gatt_rsp_t rsp = {};
        rsp.attr_value.handle = handle;
        
        // Populate response based on which characteristic is being read
        if (handle == control_char_handle) {
            uint8_t v = getControlByte();
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = v;
            ESP_LOGI(TAG, "READ control: 0x%02x", v);
        } else if (handle == eq_char_handle) {
            rsp.attr_value.len = 3;
            rsp.attr_value.value[0] = (uint8_t)(int8_t)eqBassDB;
            rsp.attr_value.value[1] = (uint8_t)(int8_t)eqMidDB;
            rsp.attr_value.value[2] = (uint8_t)(int8_t)eqTrebleDB;
            ESP_LOGI(TAG, "READ eq: %d,%d,%d", (int8_t)rsp.attr_value.value[0], 
                     (int8_t)rsp.attr_value.value[1], (int8_t)rsp.attr_value.value[2]);
        } else if (handle == name_char_handle) {
            size_t len = g_device_name.size();
            if (len > sizeof(rsp.attr_value.value)) len = sizeof(rsp.attr_value.value);
            memcpy(rsp.attr_value.value, g_device_name.c_str(), len);
            rsp.attr_value.len = len;
            ESP_LOGI(TAG, "READ name: '%s'", g_device_name.c_str());
        } else if (handle == fw_char_handle) {
            size_t len = strlen(FW_VER);
            if (len > sizeof(rsp.attr_value.value)) len = sizeof(rsp.attr_value.value);
            memcpy(rsp.attr_value.value, FW_VER, len);
            rsp.attr_value.len = len;
            ESP_LOGI(TAG, "READ fw: '%s'", FW_VER);
        } else if (handle == levels_char_handle) {
            // Return current levels
            auto dbToPos = [](float dB) -> int {
                int v = (int)roundf(dB + 120.0f);
                if (v < 0) v = 0;
                if (v > 120) v = 120;
                return v;
            };
            int l30 = dbToPos(smooth30_dB);
            int l60 = dbToPos(smooth60_dB);
            int l100 = dbToPos(smooth100_dB);
            int len = snprintf((char*)rsp.attr_value.value, sizeof(rsp.attr_value.value), 
                              "%d,%d,%d", l30, l60, l100);
            rsp.attr_value.len = len;
            ESP_LOGI(TAG, "READ levels: %d,%d,%d", l30, l60, l100);
        } else {
            // Unknown handle - return empty response
            rsp.attr_value.len = 0;
            ESP_LOGW(TAG, "READ unknown handle=%u", (unsigned)handle);
        }
        
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, 
                                   param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t handle = param->write.handle;
        uint8_t *data   = param->write.value;
        uint16_t len    = param->write.len;

        ESP_LOGI(TAG, "WRITE_EVT handle=%d len=%d need_rsp=%d is_prep=%d", handle, len,
                 (int)param->write.need_rsp, (int)param->write.is_prep);
        
        // Log the data bytes for debugging
        if (len > 0 && len <= 16) {
            char hex[64];
            int pos = 0;
            for (int i = 0; i < len && pos < 60; i++) {
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x ", data[i]);
            }
            ESP_LOGI(TAG, "WRITE data: %s", hex);
        }

        // Handle prepare writes (for long writes like name)
        if (param->write.is_prep) {
            if (handle == name_char_handle && len >= 1) {
                ESP_LOGI(TAG, "PREP WRITE matched: name_char (offset=%u, len=%u)", 
                         param->write.offset, len);
                if (param->write.offset == 0) {
                    applyDeviceNameWrite(data, len);
                }
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                           param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }

        // Debug: log expected handles for comparison
        ESP_LOGI(TAG, "Expected handles: ctrl=%u, eq=%u, name=%u, ota_ctrl=%u, ota_data=%u",
                 (unsigned)control_char_handle, (unsigned)eq_char_handle, 
                 (unsigned)name_char_handle, (unsigned)ota_ctrl_char_handle, 
                 (unsigned)ota_data_char_handle);

        // Handle characteristic value writes
        bool write_handled = false;
        if (handle == control_char_handle && len >= 1) {
            ESP_LOGI(TAG, "WRITE matched: control_char (handle=%u)", (unsigned)handle);
            applyControlByte(data[0]);
            write_handled = true;
        } else if (handle == eq_char_handle && len >= 3) {
            ESP_LOGI(TAG, "WRITE matched: eq_char (handle=%u)", (unsigned)handle);
            applyEqBytes(data, len);
            write_handled = true;
        } else if (handle == name_char_handle && len >= 1) {
            ESP_LOGI(TAG, "WRITE matched: name_char (handle=%u)", (unsigned)handle);
            applyDeviceNameWrite(data, len);
            write_handled = true;
        } else if (handle == ota_ctrl_char_handle && len >= 1) {
            ESP_LOGI(TAG, "WRITE matched: ota_ctrl_char (handle=%u)", (unsigned)handle);
            handleOtaCtrlWrite(data, len);
            write_handled = true;
        } else if (handle == ota_data_char_handle && len >= 1) {
            ESP_LOGI(TAG, "WRITE matched: ota_data_char (handle=%u)", (unsigned)handle);
            handleOtaDataWrite(data, len);
            write_handled = true;
        } else if (len == 2) {
            // CCCD write (2 bytes) - Android app enables notifications this way
            // We always notify when connected, so just accept and log it
            uint16_t cccd_val = data[0] | (data[1] << 8);
            ESP_LOGI(TAG, "CCCD write: handle=%u, value=0x%04x (notifications %s)", 
                     (unsigned)handle, cccd_val, (cccd_val & 0x01) ? "enabled" : "disabled");
            write_handled = true;
        } else {
            // Other unmatched write - accept it anyway
            ESP_LOGI(TAG, "WRITE unmatched handle=%u len=%u, accepting anyway", (unsigned)handle, len);
            write_handled = true;
        }

        // Send response if needed
        if (param->write.need_rsp) {
            esp_gatt_status_t status = write_handled ? ESP_GATT_OK : ESP_GATT_WRITE_NOT_PERMIT;
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                       param->write.trans_id, status, NULL);
        }

        break;
    }

    default:
        break;
    }
}

// -----------------------------------------------------------
// BT (controller + bluedroid) initialization
// -----------------------------------------------------------

static void bluetooth_init() {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));
}

// -----------------------------------------------------------
// DSP stream reader: called by A2DP library
//  - supports 16-bit and 24/32-bit PCM
//  - applies EQ + bass split + control flags
//  - FIX: output ALWAYS 32-bit I2S, so we never need reinstall on codec changes
// -----------------------------------------------------------

// Audio queue and fixed buffer pool to decouple BT decode from I2S
QueueHandle_t g_audio_queue = NULL;
QueueHandle_t g_audio_free_queue = NULL;

// Reduce pool/buffer sizes to lower latency (fewer, smaller chunks)
// Increase pool to handle large LDAC decoder output while still using PSRAM
static const int AUDIO_POOL_COUNT = 12;
static const size_t AUDIO_POOL_BUF_SIZE = 8192; // bytes per buffer

struct AudioBuf {
    uint32_t cap;
    uint32_t len;
    uint8_t bits;
    uint8_t channels;
    uint8_t reserved[2];
    uint8_t data[AUDIO_POOL_BUF_SIZE];
};

// Allocate pool in SPIRAM at runtime to avoid DRAM overflow
static AudioBuf *g_audio_pool = NULL;

// Runtime diagnostics counters
static volatile uint32_t g_stream_drop_count = 0;
static volatile uint32_t g_stream_enqueue_fail = 0;
static volatile uint32_t g_tx_short_write_count = 0;
static volatile uint32_t g_tx_write_fail = 0;
static const uint32_t LOG_EVERY = 100; // log interval

// (moved earlier)

static void stream_reader(const uint8_t *data, uint32_t len) {
    // Producer: enqueue raw decoded PCM to a dedicated audio TX task to avoid
    // blocking the Bluetooth/A2DP thread with heavy I2S writes or DSP.
    if (!g_i2s_initialized || len == 0) return;
    if (g_channels == 0) return;
    if (g_i2s_reconfig) return;

    if (g_audio_queue == NULL || g_audio_free_queue == NULL) return;

    // If incoming decoded chunk is larger than a single pool buffer, split it
    // across multiple pool buffers so we don't drop data that the decoder
    // expects to be consumed. Non-blocking: if pool exhausted we drop the
    // remainder to avoid blocking the BT/A2DP thread.
    size_t remaining = len;
    const uint8_t *ptr = data;
    while (remaining > 0) {
        AudioBuf *buf = NULL;
        if (xQueueReceive(g_audio_free_queue, &buf, 0) != pdTRUE) {
            // no free buffer available — drop the rest
            g_stream_drop_count = g_stream_drop_count + 1;
            if ((g_stream_drop_count % LOG_EVERY) == 0) {
                size_t free_heap = esp_get_free_heap_size();
                UBaseType_t free_slots = (g_audio_free_queue) ? uxQueueSpacesAvailable(g_audio_free_queue) : 0;
                UBaseType_t queued = (g_audio_queue) ? uxQueueMessagesWaiting(g_audio_queue) : 0;
                ESP_LOGW(TAG, "stream_reader: drop_count=%u free_slots=%u queued=%u free_heap=%u",
                         (unsigned)g_stream_drop_count, (unsigned)free_slots, (unsigned)queued, (unsigned)free_heap);
            }
            break;
        }

        size_t copy_len = (remaining > buf->cap) ? buf->cap : remaining;
        buf->len = (uint32_t)copy_len;
        buf->bits = g_bits_per_sample;
        buf->channels = g_channels;
        memcpy(buf->data, ptr, copy_len);

        if (xQueueSend(g_audio_queue, &buf, 0) != pdTRUE) {
            // queue full -> return buffer to free pool and stop
            g_stream_enqueue_fail = g_stream_enqueue_fail + 1;
            if ((g_stream_enqueue_fail % LOG_EVERY) == 0) {
                size_t free_heap = esp_get_free_heap_size();
                UBaseType_t free_slots = (g_audio_free_queue) ? uxQueueSpacesAvailable(g_audio_free_queue) : 0;
                UBaseType_t queued = (g_audio_queue) ? uxQueueMessagesWaiting(g_audio_queue) : 0;
                ESP_LOGW(TAG, "stream_reader: enqueue_fail=%u free_slots=%u queued=%u free_heap=%u",
                         (unsigned)g_stream_enqueue_fail, (unsigned)free_slots, (unsigned)queued, (unsigned)free_heap);
            }
            xQueueSend(g_audio_free_queue, &buf, 0);
            break;
        }

        ptr += copy_len;
        remaining -= copy_len;
    }
}

// -----------------------------------------------------------
// A2DP connection callback (optional)
// -----------------------------------------------------------

static void a2dp_connection_state_changed(esp_a2d_connection_state_t state, void *user) {
    switch (state) {
    case ESP_A2D_CONNECTION_STATE_CONNECTED:
        ESP_LOGI(TAG, "A2DP connected");
        break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        ESP_LOGI(TAG, "A2DP disconnected");
        // Reset I2S to default state for clean reconnection with new codec
        if (g_i2s_mutex) xSemaphoreTake(g_i2s_mutex, portMAX_DELAY);
        if (g_i2s_initialized) {
            i2s_stop(I2S_PORT);
            i2s_zero_dma_buffer(I2S_PORT);
            // Reset to default sample rate for next connection
            esp_err_t err = i2s_set_clk(I2S_PORT, I2S_DEFAULT_SR, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
            if (err == ESP_OK) {
                g_i2s_sample_rate = I2S_DEFAULT_SR;
                ESP_LOGI(TAG, "I2S reset to default SR=%u for next connection", I2S_DEFAULT_SR);
            }
            i2s_start(I2S_PORT);
        }
        if (g_i2s_mutex) xSemaphoreGive(g_i2s_mutex);
        
        // Reset DSP state for clean start
        g_sample_rate = I2S_DEFAULT_SR;
        lp_state = 0.0f;
        goertzelInitAll();
        break;
    case ESP_A2D_CONNECTION_STATE_CONNECTING:
        ESP_LOGI(TAG, "A2DP connecting...");
        break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
        ESP_LOGI(TAG, "A2DP disconnecting...");
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------
// A2DP audio state callback - helps with codec switching
// -----------------------------------------------------------

static void a2dp_audio_state_changed(esp_a2d_audio_state_t state, void *user) {
    switch (state) {
    case ESP_A2D_AUDIO_STATE_STARTED:
        ESP_LOGI(TAG, "A2DP audio STARTED (codec sr=%u, bits=%u, ch=%u)", 
                 (unsigned)g_sample_rate, (unsigned)g_bits_per_sample, (unsigned)g_channels);
        break;
    case ESP_A2D_AUDIO_STATE_STOPPED:
        ESP_LOGI(TAG, "A2DP audio STOPPED");
        // Clear audio queue on stop to avoid stale data from previous codec
        if (g_audio_queue) {
            AudioBuf *buf = NULL;
            while (xQueueReceive(g_audio_queue, &buf, 0) == pdTRUE) {
                if (buf && g_audio_free_queue) {
                    xQueueSend(g_audio_free_queue, &buf, 0);
                }
            }
        }
        // Zero DMA buffer for clean transition
        if (g_i2s_initialized) {
            i2s_zero_dma_buffer(I2S_PORT);
        }
        break;
    case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        ESP_LOGI(TAG, "A2DP audio SUSPENDED");
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------
// Button handling task
// -----------------------------------------------------------

static void buttons_task(void *arg) {
    ESP_LOGI(TAG, "buttons_task started");
    
    bool lastButton1State = true;
    bool button1Pressed   = false;
    uint32_t lastDebounceTime1 = 0;
    uint32_t pressStartTime1   = 0;
    const uint32_t debounceDelay1 = 25;

    bool buttonState2      = true;
    bool lastButton2State  = true;
    uint32_t lastDebounceTime2 = 0;
    const uint32_t debounceDelay2 = 25;

    while (true) {
        uint32_t now = millis32();

        // Button1
        bool reading1 = gpio_get_level((gpio_num_t)BUTTON1_GPIO);
        if (reading1 != lastButton1State) {
            lastDebounceTime1 = now;
        }
        if ((now - lastDebounceTime1) > debounceDelay1) {
            if (!button1Pressed && reading1 == 0) {
                button1Pressed = true;
                pressStartTime1 = now;
            } else if (button1Pressed && reading1 == 1) {
                button1Pressed = false;
                uint32_t pressDuration = now - pressStartTime1;
                if (pressDuration < 1000) {
                    bassBoostEnabled = !bassBoostEnabled;
                    ESP_LOGI(TAG, "Bass Boost: %s", bassBoostEnabled ? "ON" : "OFF");
                } else {
                    channelFlipEnabled = !channelFlipEnabled;
                    ESP_LOGI(TAG, "Channel Flip: %s", channelFlipEnabled ? "ON" : "OFF");
                }
                ble_update_control_char();
                saveControlToNVS();
            }
        }
        lastButton1State = reading1;

        // Button2
        bool reading2 = gpio_get_level((gpio_num_t)BUTTON2_GPIO);
        if (reading2 != lastButton2State) {
            lastDebounceTime2 = now;
        }
        if ((now - lastDebounceTime2) > debounceDelay2) {
            if (reading2 != buttonState2) {
                buttonState2 = reading2;
                if (buttonState2 == 1) {
                    bypassEnabled = !bypassEnabled;
                    ESP_LOGI(TAG, "BYPASS: %s", bypassEnabled ? "ON (raw stereo, EQ only)" : "OFF (DSP split active)");
                    ble_update_control_char();
                    saveControlToNVS();
                }
            }
        }
        lastButton2State = reading2;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// -----------------------------------------------------------
// Beat + levels task (LED + BLE levels)
// -----------------------------------------------------------

static void beat_task(void *arg) {
    uint32_t lastSpectrumPrintMs = 0;

    ESP_LOGI(TAG, "beat_task started, analysis=%d", g_enable_analysis);

    while (true) {
        uint32_t now = millis32();

        if (!otaActive) {
            // If analysis disabled (high SR), keep LED off and skip BLE spectrum
            if (!g_enable_analysis) {
                gpio_set_level((gpio_num_t)BEAT_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            float bass = g60_lin + g100_lin;

            if (bass < BASS_MIN_LEVEL) {
                bassSmooth *= 0.9f;
                bassAvg    *= 0.999f;
            } else {
                bassSmooth += BASS_SMOOTH_ALPHA * (bass - bassSmooth);
                bassAvg    += BASS_AVG_ALPHA * (bassSmooth - bassAvg);

                float ratio = (bassAvg > 1e-6f)
                              ? (bassSmooth / (bassAvg + 1e-6f))
                              : 0.0f;

                bool beatDetected = false;
                if (ratio > BASS_RATIO_THRESH && (now - lastBeatMs) > BEAT_MIN_INTERVAL_MS) {
                    beatDetected = true;
                    lastBeatMs = now;
                }

                if (beatDetected) {
                    gpio_set_level((gpio_num_t)BEAT_LED_GPIO, 1);
                    beatFlashActive = true;
                    beatFlashOffMs = now + BEAT_FLASH_DURATION_MS;
                }
            }

            if (beatFlashActive && now >= beatFlashOffMs) {
                gpio_set_level((gpio_num_t)BEAT_LED_GPIO, 0);
                beatFlashActive = false;
            }

            // BLE spectrum levels every 50ms
            if (now - lastSpectrumPrintMs >= LEVELS_UPDATE_MS) {
                lastSpectrumPrintMs = now;

                float d30  = g30_dB;
                float d60  = g60_dB;
                float d100 = g100_dB;

                smooth30_dB  += FFT_SMOOTH_ALPHA * (d30  - smooth30_dB);
                smooth60_dB  += FFT_SMOOTH_ALPHA * (d60  - smooth60_dB);
                smooth100_dB += FFT_SMOOTH_ALPHA * (d100 - smooth100_dB);

                auto dbToPos = [](float dB) -> int {
                    int v = (int)roundf((dB + 120.0f) * 0.67f);
                    if (v < 0) v = 0;
                    if (v > 80) v = 80;
                    return v;
                };

                int l30  = dbToPos(smooth30_dB);
                int l60  = dbToPos(smooth60_dB);
                int l100 = dbToPos(smooth100_dB);

                if (bleClientConnected) {
                    ble_update_levels_char(l30, l60, l100);
                }
            }
        } else {
            gpio_set_level((gpio_num_t)BEAT_LED_GPIO, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// (audio pool and queue defined earlier)

static void audio_tx_task(void *arg) {
    while (true) {
        AudioBuf *buf = NULL;
        if (xQueueReceive(g_audio_queue, &buf, portMAX_DELAY) == pdTRUE && buf != NULL) {
            // Convert and write to I2S (32-bit stereo container)
            uint32_t len = buf->len;
            uint8_t bits = buf->bits;
            uint8_t channels = buf->channels ? buf->channels : 2;

            uint32_t in_bytes_per_sample = (bits <= 16) ? 2u : 4u;
            uint32_t in_bytes_per_frame = in_bytes_per_sample * channels;
            uint32_t frames = (in_bytes_per_frame == 0) ? 0 : (len / in_bytes_per_frame);
            if (frames > 0) {
                if (frames > DSP_OUT_FRAMES) frames = DSP_OUT_FRAMES;

                // Cache control flags for this buffer (avoid race conditions)
                bool doBassBoost = bassBoostEnabled;
                bool doFlip = channelFlipEnabled;
                bool doBypass = bypassEnabled;
                bool doAnalysis = g_enable_analysis;

                if (in_bytes_per_sample == 2) {
                    const int16_t *smp = (const int16_t *)buf->data;
                    for (uint32_t i = 0; i < frames; ++i) {
                        int16_t l_raw = 0, r_raw = 0;
                        if (channels == 1) {
                            l_raw = smp[i]; r_raw = l_raw;
                        } else {
                            l_raw = smp[i * channels + 0];
                            r_raw = smp[i * channels + 1];
                        }
                        
                        // Convert to float for DSP (-1.0 to 1.0 range)
                        float L = (float)l_raw / 32768.0f;
                        float R = (float)r_raw / 32768.0f;
                        
                        // EQ always applies (regardless of bypass)
                        applyEQ(L, R);
                        
                        // Bass boost always applies (regardless of bypass)
                        if (doBassBoost) {
                            float mono = (L + R) * 0.5f;
                            lp_state += lp_alpha * (mono - lp_state);
                            float bass = lp_state;
                            float boostedBass = processSample(bassShelfL, bass) * BASS_GAIN_BOOST;
                            float highPart = mono - bass;
                            float mixed = boostedBass + highPart * TREBLE_GAIN;
                            L = mixed;
                            R = mixed;
                        }
                        
                        // Goertzel analysis BEFORE crossover (so we analyze full audio, not filtered)
                        if (doAnalysis) {
                            float mono = (L + R) * 0.5f;
                            goertzelProcessSample(mono);
                        }
                        
                        // Crossover split-ear mode only when NOT bypassed
                        // Channel flip controls which ear gets LP vs HP
                        if (!doBypass) {
                            float mono = (L + R) * 0.5f; // Use mono for crossover source
                            if (!doFlip) {
                                // Normal: Left = LP 90Hz, Right = HP 500Hz
                                L = processSample(crossoverLPL, mono);
                                R = processSample(crossoverHPR, mono);
                            } else {
                                // Flipped: Left = HP 500Hz, Right = LP 90Hz
                                L = processSample(crossoverHPL, mono);
                                R = processSample(crossoverLPR, mono);
                            }
                        }
                        
                        // Convert to 32-bit output with clipping
                        double scaledL = (double)L * 2147483647.0;
                        double scaledR = (double)R * 2147483647.0;
                        if (scaledL >  2147483647.0) scaledL =  2147483647.0;
                        if (scaledL < -2147483648.0) scaledL = -2147483648.0;
                        if (scaledR >  2147483647.0) scaledR =  2147483647.0;
                        if (scaledR < -2147483648.0) scaledR = -2147483648.0;
                        dsp_out32[2 * i + 0] = (int32_t)scaledL;
                        dsp_out32[2 * i + 1] = (int32_t)scaledR;
                    }
                } else {
                    // 24/32-bit input
                    const int32_t *smp = (const int32_t *)buf->data;
                    for (uint32_t i = 0; i < frames; ++i) {
                        int32_t l_raw = 0, r_raw = 0;
                        if (channels == 1) { l_raw = smp[i]; r_raw = l_raw; }
                        else { l_raw = smp[i * channels + 0]; r_raw = smp[i * channels + 1]; }
                        
                        // Convert to float for DSP (-1.0 to 1.0 range, assuming 24-bit in 32-bit container)
                        float L = (float)l_raw / 2147483648.0f;
                        float R = (float)r_raw / 2147483648.0f;
                        
                        // EQ always applies (regardless of bypass)
                        applyEQ(L, R);
                        
                        // Bass boost always applies (regardless of bypass)
                        if (doBassBoost) {
                            float mono = (L + R) * 0.5f;
                            lp_state += lp_alpha * (mono - lp_state);
                            float bass = lp_state;
                            float boostedBass = processSample(bassShelfL, bass) * BASS_GAIN_BOOST;
                            float highPart = mono - bass;
                            float mixed = boostedBass + highPart * TREBLE_GAIN;
                            L = mixed;
                            R = mixed;
                        }
                        
                        // Goertzel analysis BEFORE crossover (so we analyze full audio, not filtered)
                        if (doAnalysis) {
                            float mono = (L + R) * 0.5f;
                            goertzelProcessSample(mono);
                        }
                        
                        // Crossover split-ear mode only when NOT bypassed
                        // Channel flip controls which ear gets LP vs HP
                        if (!doBypass) {
                            float mono = (L + R) * 0.5f; // Use mono for crossover source
                            if (!doFlip) {
                                // Normal: Left = LP 90Hz, Right = HP 500Hz
                                L = processSample(crossoverLPL, mono);
                                R = processSample(crossoverHPR, mono);
                            } else {
                                // Flipped: Left = HP 500Hz, Right = LP 90Hz
                                L = processSample(crossoverHPL, mono);
                                R = processSample(crossoverLPR, mono);
                            }
                        }
                        
                        // Convert back to 32-bit
                        double scaledL = (double)L * 2147483647.0;
                        double scaledR = (double)R * 2147483647.0;
                        if (scaledL >  2147483647.0) scaledL =  2147483647.0;
                        if (scaledL < -2147483648.0) scaledL = -2147483648.0;
                        if (scaledR >  2147483647.0) scaledR =  2147483647.0;
                        if (scaledR < -2147483648.0) scaledR = -2147483648.0;
                        dsp_out32[2 * i + 0] = (int32_t)scaledL;
                        dsp_out32[2 * i + 1] = (int32_t)scaledR;
                    }
                }

                size_t bytes_to_write = frames * 2u * sizeof(int32_t);
                size_t written = 0;
                // protect I2S with mutex to avoid concurrent reconfig
                if (g_i2s_mutex) xSemaphoreTake(g_i2s_mutex, portMAX_DELAY);
                i2s_write(I2S_PORT, dsp_out32, bytes_to_write, &written, portMAX_DELAY);
                if (g_i2s_mutex) xSemaphoreGive(g_i2s_mutex);
                    if (written < bytes_to_write) {
                        g_tx_short_write_count = g_tx_short_write_count + 1;
                        if ((g_tx_short_write_count % LOG_EVERY) == 0) {
                            size_t free_heap = esp_get_free_heap_size();
                            UBaseType_t queued = (g_audio_queue) ? uxQueueMessagesWaiting(g_audio_queue) : 0;
                            ESP_LOGW(TAG, "audio_tx: short_write=%u wrote=%u expected=%u queued=%u free_heap=%u",
                                     (unsigned)g_tx_short_write_count, (unsigned)written, (unsigned)bytes_to_write, (unsigned)queued, (unsigned)free_heap);
                        }
                    }
            }

            // return buffer to free pool
            xQueueSend(g_audio_free_queue, &buf, 0);
        }
    }
}

// -----------------------------------------------------------
// app_main
// -----------------------------------------------------------

extern "C" void app_main(void) {
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Booting BDK speaker (ESP-IDF + A2DP + BLE GATT + DSP)...");

    // Load settings
    loadSettingsFromNVS();
    
    // Initialize characteristic value buffers with loaded settings
    snprintf(name_char_value, sizeof(name_char_value), "%s", g_device_name.c_str());
    snprintf(fw_char_value, sizeof(fw_char_value), "%s", FW_VER);
    control_char_value[0] = getControlByte();  // Initialize control byte from loaded settings
    eq_char_value[0] = (uint8_t)(int8_t)eqBassDB;
    eq_char_value[1] = (uint8_t)(int8_t)eqMidDB;
    eq_char_value[2] = (uint8_t)(int8_t)eqTrebleDB;
    
    ESP_LOGI(TAG, "Initial BLE values: ctrl=0x%02x, EQ=[%d,%d,%d], name='%s', fw='%s'",
             control_char_value[0], (int8_t)eq_char_value[0], (int8_t)eq_char_value[1], (int8_t)eq_char_value[2],
             name_char_value, fw_char_value);

    // GPIO init
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << BUTTON1_GPIO) | (1ULL << BUTTON2_GPIO);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    gpio_config_t io_led = {};
    io_led.intr_type = GPIO_INTR_DISABLE;
    io_led.mode = GPIO_MODE_OUTPUT;
    io_led.pin_bit_mask = (1ULL << BEAT_LED_GPIO);
    io_led.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_led.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_led);
    gpio_set_level((gpio_num_t)BEAT_LED_GPIO, 0);

    // Prepare UUIDs
    uuid128_from_string(BLE_SERVICE_UUID_LEVELS,  uuid_levels_service);
    uuid128_from_string(BLE_CHAR_UUID_LEVELS,     uuid_levels_char);
    uuid128_from_string(BLE_SERVICE_UUID_CONTROL, uuid_control_service);
    uuid128_from_string(BLE_CHAR_UUID_CONTROL,    uuid_control_char);
    uuid128_from_string(BLE_CHAR_UUID_EQ,         uuid_eq_char);
    uuid128_from_string(BLE_CHAR_UUID_DEVNAME,    uuid_name_char);
    uuid128_from_string(BLE_CHAR_UUID_FWVER,      uuid_fw_char);
    uuid128_from_string(BLE_CHAR_UUID_OTA_CTRL,   uuid_ota_ctrl_char);
    uuid128_from_string(BLE_CHAR_UUID_OTA_DATA,   uuid_ota_data_char);

    // Init analysis filters for default SR
    update_filter_params();
    updateEQFilters();
    goertzelInitAll();

    // Bluetooth (controller + BLE GATT)
    bluetooth_init();

    // FIX: Create I2S mutex
    g_i2s_mutex = xSemaphoreCreateMutex();
    if (!g_i2s_mutex) {
        ESP_LOGE(TAG, "Failed to create I2S mutex");
    }

    // Create audio queue and free-pool queue and start audio TX task
    g_audio_queue = xQueueCreate(AUDIO_POOL_COUNT, sizeof(void*));
    g_audio_free_queue = xQueueCreate(AUDIO_POOL_COUNT, sizeof(void*));
    if (!g_audio_queue || !g_audio_free_queue) {
        ESP_LOGE(TAG, "Failed to create audio queues");
    } else {
        // Allocate pool in SPIRAM to avoid DRAM overflow
        g_audio_pool = (AudioBuf*)heap_caps_malloc(sizeof(AudioBuf) * (size_t)AUDIO_POOL_COUNT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_audio_pool) {
            ESP_LOGW(TAG, "Failed to allocate audio pool in SPIRAM, trying internal heap (may overflow DRAM)");
            g_audio_pool = (AudioBuf*)heap_caps_malloc(sizeof(AudioBuf) * (size_t)AUDIO_POOL_COUNT, MALLOC_CAP_8BIT);
        }

        if (!g_audio_pool) {
            ESP_LOGE(TAG, "Failed to allocate audio pool");
        } else {
            // Allocate dsp_out32 in internal RAM for low-latency writes
            dsp_out32 = (int32_t*)heap_caps_malloc(sizeof(int32_t) * DSP_OUT_FRAMES * 2, MALLOC_CAP_INTERNAL);
            if (!dsp_out32) {
                ESP_LOGW(TAG, "Failed to allocate dsp_out32 in internal RAM, falling back to SPIRAM");
                dsp_out32 = (int32_t*)heap_caps_malloc(sizeof(int32_t) * DSP_OUT_FRAMES * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
            }
            if (!dsp_out32) {
                ESP_LOGE(TAG, "Failed to allocate dsp_out32 buffer");
            }
            // initialize pool and push all buffers to free queue
            for (int i = 0; i < AUDIO_POOL_COUNT; ++i) {
                g_audio_pool[i].cap = (uint32_t)AUDIO_POOL_BUF_SIZE;
                g_audio_pool[i].len = 0;
                g_audio_pool[i].bits = 16;
                g_audio_pool[i].channels = 2;
                void *p = &g_audio_pool[i];
                xQueueSend(g_audio_free_queue, &p, 0);
            }

            // Increase priority to favor timely audio TX and reduce backlog
            xTaskCreatePinnedToCore(audio_tx_task, "audio_tx", 8192, NULL, 10, NULL, 1);
        }
    }

    // FIX: Init I2S ONCE (32-bit stereo). Codec callback updates only the clock.
    esp_err_t i2s_err = configure_i2s_init_once(I2S_DEFAULT_SR);
    if (i2s_err != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed, cannot start A2DP audio: %s", esp_err_to_name(i2s_err));
        return;
    }

    // A2DP sink: we will manage I2S ourselves to tune DMA and avoid underruns
    a2dp_sink.set_output_active(false);
    // register stream reader (we do output via audio_tx_task)
    a2dp_sink.set_stream_reader(stream_reader, false);
    a2dp_sink.set_codec_config_callback(codec_config_cb);
    a2dp_sink.set_auto_reconnect(true);

    // ESP32: dedicate core 0 to Bluetooth (incoming) and move other tasks to core 1
    a2dp_sink.set_task_core(0);

    a2dp_sink.set_on_connection_state_changed(a2dp_connection_state_changed);
    a2dp_sink.set_on_audio_state_changed(a2dp_audio_state_changed);

    a2dp_sink.start(g_device_name.c_str());
    ESP_LOGI(TAG, "A2DP sink started as '%s'", g_device_name.c_str());

    // Start UI tasks - smaller stack (2048) since internal RAM is limited
    BaseType_t ret1 = xTaskCreate(buttons_task, "buttons_task", 2048, nullptr, 5, nullptr);
    BaseType_t ret2 = xTaskCreate(beat_task,    "beat_task",    2048, nullptr, 4, nullptr);

    if (ret1 != pdPASS || ret2 != pdPASS) {
        ESP_LOGE(TAG, "Task creation FAILED: buttons=%d, beat=%d", ret1, ret2);
    }

    ESP_LOGI(TAG, "System ready: buttons, BLE GATT, OTA, DSP active");
}
