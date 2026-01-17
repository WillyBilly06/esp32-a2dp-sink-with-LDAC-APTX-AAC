#pragma once

// -----------------------------------------------------------
// Encoder Controller - Manages Quad Rotary Encoder Breakout (5752)
// Simple approach matching Arduino example exactly
// -----------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "seesaw_encoder.h"

// I2C configuration
#ifdef CONFIG_ENCODER_I2C_SDA_GPIO
    #define ENCODER_I2C_SDA_GPIO  ((gpio_num_t)CONFIG_ENCODER_I2C_SDA_GPIO)
#else
    #define ENCODER_I2C_SDA_GPIO  GPIO_NUM_23
#endif

#ifdef CONFIG_ENCODER_I2C_SCL_GPIO
    #define ENCODER_I2C_SCL_GPIO  ((gpio_num_t)CONFIG_ENCODER_I2C_SCL_GPIO)
#else
    #define ENCODER_I2C_SCL_GPIO  GPIO_NUM_22
#endif

#ifdef CONFIG_ENCODER_ADDR_QUAD
    #define ENCODER_QUAD_ADDR  CONFIG_ENCODER_ADDR_QUAD
#else
    #define ENCODER_QUAD_ADDR  0x36  // SEESAW_QUAD_DEFAULT_ADDR
#endif

#define ENCODER_I2C_PORT      I2C_NUM_1
#define ENCODER_I2C_FREQ_HZ   400000  // 400kHz standard
#define ENCODER_POLL_MS       25  // 25ms polling (reduced from 10ms to avoid audio interference)

// Encoder indices
#define ENC_VOLUME  0
#define ENC_BASS    1
#define ENC_MID     2
#define ENC_TREBLE  3

// Volume range (0-127 per Bluetooth A2DP spec)
#define VOLUME_MIN  0
#define VOLUME_MAX  127
#define VOLUME_STEP 3

// EQ limits (dB)
#define EQ_MIN     -12
#define EQ_MAX      12
#define EQ_STEP     1

static const char* ENC_TAG = "EncoderCtrl";

// Callbacks
typedef void (*VolumeChangedCb)(uint8_t volume);
typedef void (*PlayPauseCb)();
typedef void (*EqChangedCb)(int8_t bass, int8_t mid, int8_t treble);
typedef void (*BrightnessChangedCb)(uint8_t brightness);

class EncoderController {
public:
    static EncoderController& getInstance() {
        static EncoderController instance;
        return instance;
    }

    bool init() {
        if (m_initialized) return true;
        
        // Initialize I2C
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = ENCODER_I2C_SDA_GPIO;
        conf.scl_io_num = ENCODER_I2C_SCL_GPIO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = ENCODER_I2C_FREQ_HZ;
        
        esp_err_t err = i2c_param_config(ENCODER_I2C_PORT, &conf);
        if (err != ESP_OK) {
            ESP_LOGE(ENC_TAG, "I2C config failed: %s", esp_err_to_name(err));
            return false;
        }
        
        err = i2c_driver_install(ENCODER_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(ENC_TAG, "I2C driver install failed: %s", esp_err_to_name(err));
            return false;
        }
        
        ESP_LOGI(ENC_TAG, "I2C initialized on SDA=%d, SCL=%d", 
                 ENCODER_I2C_SDA_GPIO, ENCODER_I2C_SCL_GPIO);
        
        // Initialize quad encoder (like Arduino ss.begin())
        if (!m_encoder.begin(ENCODER_I2C_PORT, ENCODER_QUAD_ADDR)) {
            ESP_LOGW(ENC_TAG, "Quad encoder not found at 0x%02X", ENCODER_QUAD_ADDR);
        } else {
            ESP_LOGI(ENC_TAG, "Quad encoder initialized at 0x%02X", ENCODER_QUAD_ADDR);
            
            // Set NeoPixel colors
            m_encoder.setPixelColor(ENC_VOLUME, 0, 100, 0);    // Green
            m_encoder.setPixelColor(ENC_BASS, 100, 0, 0);      // Red
            m_encoder.setPixelColor(ENC_MID, 0, 0, 100);       // Blue
            m_encoder.setPixelColor(ENC_TREBLE, 100, 100, 0);  // Yellow
            m_encoder.showPixels();
            
            // Store initial positions
            for (int e = 0; e < 4; e++) {
                m_lastPos[e] = m_encoder.getPosition(e);
            }
        }
        
        m_initialized = true;
        return true;
    }
    
    // Set callbacks
    void setVolumeCallback(VolumeChangedCb cb) { m_volumeCb = cb; }
    void setPlayPauseCallback(PlayPauseCb cb) { m_playPauseCb = cb; }
    void setEqCallback(EqChangedCb cb) { m_eqCb = cb; }
    void setBrightnessCallback(BrightnessChangedCb cb) { m_brightnessCb = cb; }
    
    // Set current values
    void setCurrentVolume(uint8_t volume) { m_volume = (volume <= VOLUME_MAX) ? volume : VOLUME_MAX; }
    void setCurrentEq(int8_t bass, int8_t mid, int8_t treble) { m_bass = bass; m_mid = mid; m_treble = treble; }
    void setCurrentBrightness(uint8_t brightness) {
        m_brightness = brightness;
    }
    
    // Poll - call this regularly (like Arduino loop)
    void poll() {
        if (!m_initialized || !m_encoder.isInitialized()) return;
        
        // Poll encoder (reads all positions)
        m_encoder.poll();
        
        bool eqChanged = false;
        
        // ------ Check buttons with debounce ------
        
        // Volume button = play/pause (with debounce)
        bool volBtnRaw = m_encoder.isButtonPressed(ENC_VOLUME);
        if (volBtnRaw) {
            if (m_btnDebounce[ENC_VOLUME] < DEBOUNCE_COUNT) m_btnDebounce[ENC_VOLUME]++;
        } else {
            m_btnDebounce[ENC_VOLUME] = 0;
        }
        bool volBtn = (m_btnDebounce[ENC_VOLUME] >= DEBOUNCE_COUNT);
        if (volBtn && !m_lastBtnState[ENC_VOLUME]) {
            if (m_playPauseCb) m_playPauseCb();
            ESP_LOGI(ENC_TAG, "Play/Pause pressed");
        }
        m_lastBtnState[ENC_VOLUME] = volBtn;
        
        // Bass button = brightness cycle (with debounce)
        bool bassBtnRaw = m_encoder.isButtonPressed(ENC_BASS);
        if (bassBtnRaw) {
            if (m_btnDebounce[ENC_BASS] < DEBOUNCE_COUNT) m_btnDebounce[ENC_BASS]++;
        } else {
            m_btnDebounce[ENC_BASS] = 0;
        }
        bool bassBtn = (m_btnDebounce[ENC_BASS] >= DEBOUNCE_COUNT);
        if (bassBtn && !m_lastBtnState[ENC_BASS]) {
            m_brightnessMode = !m_brightnessMode;
            if (m_brightnessMode) {
                // Enter brightness mode - change bass encoder LED to white
                m_encoder.setPixelColor(ENC_BASS, 100, 100, 100);  // White = brightness mode
                ESP_LOGI(ENC_TAG, "Brightness mode: ON (rotate bass encoder to adjust, press again to confirm)");
            } else {
                // Exit brightness mode - restore bass encoder LED to red
                m_encoder.setPixelColor(ENC_BASS, 100, 0, 0);  // Red = normal
                ESP_LOGI(ENC_TAG, "Brightness mode: OFF - saved brightness: %d", m_brightness);
                // Notify brightness change when exiting
                if (m_brightnessCb) m_brightnessCb(m_brightness);
            }
            m_encoder.showPixels();
        }
        m_lastBtnState[ENC_BASS] = bassBtn;
        
        // ------ Check encoders (like Arduino: if (enc_positions[e] != new_position)) ------
        
        // Volume encoder
        if (m_encoder.hasChanged(ENC_VOLUME)) {
            int32_t newPos = m_encoder.getPosition(ENC_VOLUME);
            int32_t delta = newPos - m_lastPos[ENC_VOLUME];
            m_lastPos[ENC_VOLUME] = newPos;
            
            if (delta != 0) {
                int newVol = (int)m_volume + (delta * VOLUME_STEP);
                if (newVol < VOLUME_MIN) newVol = VOLUME_MIN;
                if (newVol > VOLUME_MAX) newVol = VOLUME_MAX;
                m_volume = (uint8_t)newVol;
                
                if (m_volumeCb) m_volumeCb(m_volume);
                ESP_LOGI(ENC_TAG, "Volume: %d (pos: %ld)", m_volume, (long)newPos);
            }
        }
        
        // Bass encoder - adjust brightness in brightness mode, otherwise adjust bass EQ
        if (m_encoder.hasChanged(ENC_BASS)) {
            int32_t newPos = m_encoder.getPosition(ENC_BASS);
            int32_t delta = newPos - m_lastPos[ENC_BASS];
            m_lastPos[ENC_BASS] = newPos;
            
            if (delta != 0) {
                if (m_brightnessMode) {
                    // Brightness adjustment mode
                    int newBright = (int)m_brightness + (delta * BRIGHTNESS_STEP);
                    if (newBright < BRIGHTNESS_MIN) newBright = BRIGHTNESS_MIN;
                    if (newBright > BRIGHTNESS_MAX) newBright = BRIGHTNESS_MAX;
                    m_brightness = (uint8_t)newBright;
                    
                    // Immediate feedback - notify brightness change while adjusting
                    if (m_brightnessCb) m_brightnessCb(m_brightness);
                    ESP_LOGI(ENC_TAG, "Brightness: %d (pos: %ld)", m_brightness, (long)newPos);
                } else {
                    // Normal bass EQ mode
                    m_bass = clampEq(m_bass + delta * EQ_STEP);
                    eqChanged = true;
                    ESP_LOGI(ENC_TAG, "Bass: %d (pos: %ld)", m_bass, (long)newPos);
                }
            }
        }
        
        // Mid encoder
        if (m_encoder.hasChanged(ENC_MID)) {
            int32_t newPos = m_encoder.getPosition(ENC_MID);
            int32_t delta = newPos - m_lastPos[ENC_MID];
            m_lastPos[ENC_MID] = newPos;
            
            if (delta != 0) {
                m_mid = clampEq(m_mid + delta * EQ_STEP);
                eqChanged = true;
                ESP_LOGI(ENC_TAG, "Mid: %d (pos: %ld)", m_mid, (long)newPos);
            }
        }
        
        // Treble encoder
        if (m_encoder.hasChanged(ENC_TREBLE)) {
            int32_t newPos = m_encoder.getPosition(ENC_TREBLE);
            int32_t delta = newPos - m_lastPos[ENC_TREBLE];
            m_lastPos[ENC_TREBLE] = newPos;
            
            if (delta != 0) {
                m_treble = clampEq(m_treble + delta * EQ_STEP);
                eqChanged = true;
                ESP_LOGI(ENC_TAG, "Treble: %d (pos: %ld)", m_treble, (long)newPos);
            }
        }
        
        // Notify EQ change
        if (eqChanged && m_eqCb) {
            m_eqCb(m_bass, m_mid, m_treble);
        }
    }
    
    // Getters
    uint8_t getVolume() const { return m_volume; }
    int8_t getBass() const { return m_bass; }
    int8_t getMid() const { return m_mid; }
    int8_t getTreble() const { return m_treble; }
    uint8_t getBrightness() const { return m_brightness; }
    
private:
    EncoderController() = default;
    
    static int8_t clampEq(int val) {
        if (val < EQ_MIN) return EQ_MIN;
        if (val > EQ_MAX) return EQ_MAX;
        return (int8_t)val;
    }
    
    bool m_initialized = false;
    SeesawQuadEncoder m_encoder;
    
    // Last positions (like Arduino enc_positions[4])
    int32_t m_lastPos[4] = {0, 0, 0, 0};
    
    // Last button states
    bool m_lastBtnState[4] = {false, false, false, false};
    uint8_t m_btnDebounce[4] = {0, 0, 0, 0};  // Debounce counters
    static constexpr uint8_t DEBOUNCE_COUNT = 3;  // Require 3 consecutive reads
    
    // Brightness adjustment mode
    bool m_brightnessMode = false;
    static constexpr int BRIGHTNESS_STEP = 5;  // Step for encoder brightness adjustment
    static constexpr uint8_t BRIGHTNESS_MIN = 5;
    static constexpr uint8_t BRIGHTNESS_MAX = 255;
    
    // Current values
    uint8_t m_volume = 64;
    int8_t m_bass = 0;
    int8_t m_mid = 0;
    int8_t m_treble = 0;
    uint8_t m_brightness = 100;
    
    // Callbacks
    VolumeChangedCb m_volumeCb = nullptr;
    PlayPauseCb m_playPauseCb = nullptr;
    EqChangedCb m_eqCb = nullptr;
    BrightnessChangedCb m_brightnessCb = nullptr;
};

// Encoder task - runs on core 0 to avoid interfering with audio on core 1
inline void encoderTask(void* param) {
    EncoderController& enc = EncoderController::getInstance();
    while (true) {
        enc.poll();
        taskYIELD();  // Give other tasks a chance
        vTaskDelay(pdMS_TO_TICKS(ENCODER_POLL_MS));
    }
}

// Start encoder task - priority 2 (below audio tasks), pinned to core 0
inline void startEncoderTask(int priority = 2, int stackSize = 4096) {
    if (!EncoderController::getInstance().init()) {
        ESP_LOGW(ENC_TAG, "Encoder init failed, not starting task");
        return;
    }
    xTaskCreatePinnedToCore(encoderTask, "encoder", stackSize, nullptr, priority, nullptr, 0);
    ESP_LOGI(ENC_TAG, "Encoder task started on core 0, priority %d", priority);
}
