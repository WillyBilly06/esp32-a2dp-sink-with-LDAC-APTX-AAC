#pragma once

// -----------------------------------------------------------
// LED Controller
// Manages effects, demo mode switching, and audio integration
// Uses SPI DMA driver for reliable LED output
// -----------------------------------------------------------

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "led_config.h"
#include "led_driver_spi.h"
#include "led_effects.h"
#include "../dsp/dsp_processor.h"

static const char* LED_TAG = "LedController";

class LedController {
public:
    static LedController& getInstance() {
        static LedController instance;
        return instance;
    }
    
    bool init(gpio_num_t gpioPin, uint8_t brightness = LED_DEFAULT_BRIGHTNESS) {
        m_brightness = brightness;
        
        if (m_driver.init(gpioPin) != ESP_OK) {
            ESP_LOGE(LED_TAG, "Failed to initialize LED driver");
            return false;
        }
        
        // Create all effects
        createEffects();
        
        // Load saved effect from NVS
        loadSettings();
        
        // Set initial effect
        if (m_currentEffect < LED_EFFECT_COUNT && m_effects[m_currentEffect]) {
            m_effects[m_currentEffect]->init(&m_driver);
        }
        
        m_initialized = true;
        ESP_LOGI(LED_TAG, "LED Controller initialized, effect: %d", m_currentEffect);
        return true;
    }
    
    void update(float bass, float mid, float high, 
                float bassDb, float midDb, float highDb,
                bool beat, float beatIntensity, bool audioPlaying) {
        if (!m_initialized) return;
        
        // Track audio activity
        if (audioPlaying) {
            m_lastAudioTime = xTaskGetTickCount();
            m_inDemoMode = false;
        } else {
            // Check if we should switch to demo mode
            TickType_t elapsed = xTaskGetTickCount() - m_lastAudioTime;
            if (elapsed > pdMS_TO_TICKS(m_demoTimeoutMs) && !m_inDemoMode) {
                m_inDemoMode = true;
                ESP_LOGI(LED_TAG, "Switching to demo mode");
            }
        }
        
        // Update current effect
        LedEffect* effect = getCurrentEffect();
        if (!effect) return;
        
        if (m_inDemoMode) {
            effect->updateDemo();
        } else {
            AudioData audio;
            audio.bass = bass;
            audio.mid = mid;
            audio.high = high;
            audio.bassDB = bassDb;
            audio.midDB = midDb;
            audio.highDB = highDb;
            audio.beat = beat;
            audio.beatIntensity = beatIntensity;
            audio.audioActive = audioPlaying;
            effect->update(audio);
        }
        
        // Apply global brightness and show
        m_driver.setBrightness(m_brightness);
        m_driver.show();
    }
    
    void nextEffect() {
        m_currentEffect = (m_currentEffect + 1) % LED_EFFECT_COUNT;
        
        LedEffect* effect = getCurrentEffect();
        if (effect) {
            effect->init(&m_driver);
            ESP_LOGI(LED_TAG, "Effect changed to: %s (%d)", effect->getName(), m_currentEffect);
        }
        
        saveSettings();
    }
    
    void previousEffect() {
        if (m_currentEffect == 0) {
            m_currentEffect = LED_EFFECT_COUNT - 1;
        } else {
            m_currentEffect--;
        }
        
        LedEffect* effect = getCurrentEffect();
        if (effect) {
            effect->init(&m_driver);
            ESP_LOGI(LED_TAG, "Effect changed to: %s (%d)", effect->getName(), m_currentEffect);
        }
        
        saveSettings();
    }
    
    void setEffect(int effectId) {
        if (effectId >= 0 && effectId < LED_EFFECT_COUNT) {
            m_currentEffect = effectId;
            LedEffect* effect = getCurrentEffect();
            if (effect) {
                effect->init(&m_driver);
            }
            saveSettings();
        }
    }
    
    int getCurrentEffectId() const { return m_currentEffect; }
    
    const char* getCurrentEffectName() const {
        LedEffect* effect = const_cast<LedController*>(this)->getCurrentEffect();
        return effect ? effect->getName() : "Unknown";
    }
    
    void setBrightness(uint8_t brightness) {
        m_brightness = brightness;
    }
    
    uint8_t getBrightness() const { return m_brightness; }
    
    void setDemoTimeout(uint32_t timeoutMs) {
        m_demoTimeoutMs = timeoutMs;
    }
    
    bool isInDemoMode() const { return m_inDemoMode; }
    
    void forceDemoMode(bool demo) {
        m_inDemoMode = demo;
        if (!demo) {
            m_lastAudioTime = xTaskGetTickCount();
        }
    }
    
    // OTA progress display mode
    void setOtaMode(bool enabled) {
        m_otaMode = enabled;
        m_otaProgress = 0;
        if (enabled) {
            ESP_LOGI(LED_TAG, "LED OTA mode enabled");
        }
    }
    
    void setOtaProgress(uint8_t percent) {
        if (percent > 100) percent = 100;
        m_otaProgress = percent;
    }
    
    bool isOtaMode() const { return m_otaMode; }
    
    // Render OTA progress bar on the LED matrix
    void renderOtaProgress() {
        if (!m_initialized) return;
        
        m_driver.clear();
        
        // Calculate how many LEDs to light up (256 total, map 0-100% to 0-256)
        int totalLeds = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT;
        int litLeds = (m_otaProgress * totalLeds) / 100;
        
        // Pulsing brightness effect using frame counter
        static uint32_t otaFrame = 0;
        uint8_t pulse = sin8((otaFrame * 4) & 0xFF);
        uint8_t baseBrightness = 180 + (pulse >> 2);  // 180-243 range
        otaFrame++;
        
        // Fill from bottom to top, left to right
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            int yFlip = LED_MATRIX_HEIGHT - 1 - y;  // Start from bottom
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                int ledIndex = y * LED_MATRIX_WIDTH + x;
                
                if (ledIndex < litLeds) {
                    // Color gradient from green (bottom) to cyan (middle) to blue (top)
                    uint8_t hue;
                    if (y < LED_MATRIX_HEIGHT / 3) {
                        hue = 96;  // Green
                    } else if (y < 2 * LED_MATRIX_HEIGHT / 3) {
                        hue = 128;  // Cyan
                    } else {
                        hue = 160;  // Blue
                    }
                    
                    m_driver.setPixelXY(x, yFlip, RGB::fromHSV(hue, 255, baseBrightness));
                } else if (ledIndex == litLeds) {
                    // Leading edge - bright white
                    m_driver.setPixelXY(x, yFlip, RGB(255, 255, 255));
                } else {
                    // Background - very dim
                    m_driver.setPixelXY(x, yFlip, RGB(2, 2, 2));
                }
            }
        }
        
        m_driver.setBrightness(m_brightness);
        m_driver.show();
    }
    
    LedDriver& getDriver() { return m_driver; }
    
private:
    LedController() = default;
    ~LedController() {
        for (int i = 0; i < LED_EFFECT_COUNT; i++) {
            delete m_effects[i];
        }
    }
    
    LedController(const LedController&) = delete;
    LedController& operator=(const LedController&) = delete;
    
    void createEffects() {
        m_effects[LED_EFFECT_SPECTRUM_BARS] = new SpectrumBarsEffect();
        m_effects[LED_EFFECT_BEAT_PULSE] = new BeatPulseEffect();
        m_effects[LED_EFFECT_RIPPLE] = new RippleEffect();
        m_effects[LED_EFFECT_FIRE] = new FireEffect();
        m_effects[LED_EFFECT_PLASMA] = new PlasmaEffect();
        m_effects[LED_EFFECT_RAIN] = new MatrixRainEffect();
        m_effects[LED_EFFECT_VU_METER] = new VUMeterEffect();
        m_effects[LED_EFFECT_STARFIELD] = new StarfieldEffect();
        m_effects[LED_EFFECT_WAVE] = new WaveEffect();
        m_effects[LED_EFFECT_FIREWORKS] = new FireworksEffect();
        m_effects[LED_EFFECT_RAINBOW_WAVE] = new RainbowWaveEffect();
        m_effects[LED_EFFECT_PARTICLE_BURST] = new ParticleBurstEffect();
        m_effects[LED_EFFECT_KALEIDOSCOPE] = new KaleidoscopeEffect();
        m_effects[LED_EFFECT_FREQUENCY_SPIRAL] = new FrequencySpiralEffect();
        m_effects[LED_EFFECT_BASS_REACTOR] = new BassReactorEffect();
        // New effects
        m_effects[LED_EFFECT_METEOR_SHOWER] = new MeteorShowerEffect();
        m_effects[LED_EFFECT_BREATHING] = new BreathingEffect();
        m_effects[LED_EFFECT_DNA_HELIX] = new DNAHelixEffect();
        m_effects[LED_EFFECT_AUDIO_SCOPE] = new AudioScopeEffect();
        m_effects[LED_EFFECT_BOUNCING_BALLS] = new BouncingBallsEffect();
        m_effects[LED_EFFECT_LAVA_LAMP] = new LavaLampEffect();
    }
    
    LedEffect* getCurrentEffect() {
        if (m_currentEffect >= 0 && m_currentEffect < LED_EFFECT_COUNT) {
            return m_effects[m_currentEffect];
        }
        return nullptr;
    }
    
    void loadSettings() {
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READONLY, &handle) == ESP_OK) {
            int32_t effect = 0;
            if (nvs_get_i32(handle, "effect", &effect) == ESP_OK) {
                if (effect >= 0 && effect < LED_EFFECT_COUNT) {
                    m_currentEffect = effect;
                }
            }
            
            uint8_t brightness = LED_DEFAULT_BRIGHTNESS;
            if (nvs_get_u8(handle, "bright", &brightness) == ESP_OK) {
                m_brightness = brightness;
            }
            
            nvs_close(handle);
            ESP_LOGI(LED_TAG, "Loaded settings: effect=%d, brightness=%d", m_currentEffect, m_brightness);
        }
    }
    
    void saveSettings() {
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READWRITE, &handle) == ESP_OK) {
            nvs_set_i32(handle, "effect", m_currentEffect);
            nvs_set_u8(handle, "bright", m_brightness);
            nvs_commit(handle);
            nvs_close(handle);
        }
    }
    
    LedDriverSPI m_driver;  // SPI DMA driver
    LedEffect* m_effects[LED_EFFECT_COUNT] = {nullptr};
    
    int m_currentEffect = LED_EFFECT_SPECTRUM_BARS;
    uint8_t m_brightness = LED_DEFAULT_BRIGHTNESS;
    bool m_initialized = false;
    
    bool m_inDemoMode = true;
    TickType_t m_lastAudioTime = 0;
    uint32_t m_demoTimeoutMs = LED_DEMO_TIMEOUT_MS;
    
    // OTA progress display
    bool m_otaMode = false;
    uint8_t m_otaProgress = 0;
};

// -----------------------------------------------------------
// LED Task for FreeRTOS
// -----------------------------------------------------------

static TaskHandle_t ledTaskHandle = nullptr;
static volatile bool ledTaskRunning = false;
static volatile bool ledBeatDetected = false;
static DSPProcessor* g_ledDsp = nullptr;

inline void setLedBeat(bool beat) { ledBeatDetected = beat; }

// Helper struct to hold audio readings - avoids optimizer issues
struct LedAudioReadings {
    float bass;
    float mid;
    float high;
    float bassDb;
    float midDb;
    float highDb;
    bool audioPlaying;
};

// -----------------------------------------------------------
// Automatic Gain Control (AGC) for LED effects
// Ultra-aggressive normalization for 1% volume support
// Uses dB-domain processing for proper low-level handling
// -----------------------------------------------------------
struct AudioAGC {
    // Running peak in dB domain (much better for quiet audio)
    float peakDbBass = -60.0f;
    float peakDbMid = -60.0f;
    float peakDbHigh = -60.0f;
    
    // AGC parameters
    float targetDb = -6.0f;      // Target output level (-6dB = 0.5 linear)
    float attackRate = 0.3f;     // Fast attack to catch beats
    float decayRate = 0.9995f;   // Very slow decay (keeps gain high for quiet parts)
    float minInputDb = -80.0f;   // Minimum detectable input (-80dB = 0.0001 linear)
    float maxGainDb = 60.0f;     // Maximum gain in dB (60dB = 1000x amplification!)
    
    void updatePeaks(float bassLin, float midLin, float highLin) {
        // Convert linear to dB with floor
        auto linToDb = [](float lin) -> float {
            if (lin < 0.000001f) return -120.0f;
            return 20.0f * log10f(lin);
        };
        
        float bassDb = linToDb(bassLin);
        float midDb = linToDb(midLin);
        float highDb = linToDb(highLin);
        
        // Update peak tracking in dB domain (handles quiet audio much better)
        if (bassDb > peakDbBass) {
            peakDbBass = peakDbBass + (bassDb - peakDbBass) * attackRate;
        } else {
            // Decay towards minimum (allows gain to increase for quiet audio)
            peakDbBass = peakDbBass * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        if (midDb > peakDbMid) {
            peakDbMid = peakDbMid + (midDb - peakDbMid) * attackRate;
        } else {
            peakDbMid = peakDbMid * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        if (highDb > peakDbHigh) {
            peakDbHigh = peakDbHigh + (highDb - peakDbHigh) * attackRate;
        } else {
            peakDbHigh = peakDbHigh * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        // Clamp peaks to valid range
        if (peakDbBass < minInputDb) peakDbBass = minInputDb;
        if (peakDbMid < minInputDb) peakDbMid = minInputDb;
        if (peakDbHigh < minInputDb) peakDbHigh = minInputDb;
    }
    
    void normalize(float& bass, float& mid, float& high) {
        // Convert input to dB
        auto linToDb = [](float lin) -> float {
            if (lin < 0.000001f) return -120.0f;
            return 20.0f * log10f(lin);
        };
        
        // Convert dB back to linear
        auto dbToLin = [](float db) -> float {
            if (db < -120.0f) return 0.0f;
            return powf(10.0f, db / 20.0f);
        };
        
        // Calculate required gain in dB to reach target
        float gainDbBass = targetDb - peakDbBass;
        float gainDbMid = targetDb - peakDbMid;
        float gainDbHigh = targetDb - peakDbHigh;
        
        // Clamp gain to maximum (60dB = 1000x)
        if (gainDbBass > maxGainDb) gainDbBass = maxGainDb;
        if (gainDbMid > maxGainDb) gainDbMid = maxGainDb;
        if (gainDbHigh > maxGainDb) gainDbHigh = maxGainDb;
        
        // Don't apply negative gain (don't attenuate loud signals for LED effects)
        if (gainDbBass < 0.0f) gainDbBass = 0.0f;
        if (gainDbMid < 0.0f) gainDbMid = 0.0f;
        if (gainDbHigh < 0.0f) gainDbHigh = 0.0f;
        
        // Apply gain in dB domain then convert back to linear
        float bassDb = linToDb(bass) + gainDbBass;
        float midDb = linToDb(mid) + gainDbMid;
        float highDb = linToDb(high) + gainDbHigh;
        
        bass = dbToLin(bassDb);
        mid = dbToLin(midDb);
        high = dbToLin(highDb);
        
        // Clamp to 0-1 range
        if (bass > 1.0f) bass = 1.0f;
        if (mid > 1.0f) mid = 1.0f;
        if (high > 1.0f) high = 1.0f;
    }
};

static AudioAGC g_agc;  // Global AGC instance

// Non-inline helper to read DSP data with AGC - helps avoid GCC ICE
static void __attribute__((noinline)) readDspData(LedAudioReadings& r) {
    r.bass = 0.0f;
    r.mid = 0.0f;
    r.high = 0.0f;
    r.bassDb = -60.0f;
    r.midDb = -60.0f;
    r.highDb = -60.0f;
    r.audioPlaying = false;
    
    if (g_ledDsp) {
        r.bassDb = g_ledDsp->getGoertzel30dB();
        r.midDb = g_ledDsp->getGoertzel60dB();
        r.highDb = g_ledDsp->getGoertzel100dB();
        
        // Read raw linear levels
        float rawBass = g_ledDsp->getGoertzel30Lin() + g_ledDsp->getGoertzel60Lin();
        float rawMid = g_ledDsp->getGoertzel100Lin();
        float rawHigh = g_ledDsp->getPeakLin(2);
        
        if (rawBass > 1.0f) rawBass = 1.0f;
        if (rawMid > 1.0f) rawMid = 1.0f;
        if (rawHigh > 1.0f) rawHigh = 1.0f;
        
        // Update AGC peak tracking
        g_agc.updatePeaks(rawBass, rawMid, rawHigh);
        
        // Apply AGC normalization
        r.bass = rawBass;
        r.mid = rawMid;
        r.high = rawHigh;
        g_agc.normalize(r.bass, r.mid, r.high);
        
        // Very low threshold for audio detection (-90dB for 1% volume support)
        r.audioPlaying = (r.bassDb > -90.0f || r.midDb > -90.0f || r.highDb > -90.0f);
    }
}

static void ledTask(void* param) {
    ESP_LOGI(LED_TAG, ">>> LED TASK ENTRY <<<");
    
    // Delay briefly to let other startup logs clear
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(LED_TAG, "LED task starting on core %d...", xPortGetCoreID());
    
    LedController& controller = LedController::getInstance();
    
    // Initialize with configured GPIO and brightness
    #ifdef CONFIG_LED_MATRIX_GPIO
        int gpio = CONFIG_LED_MATRIX_GPIO;
    #else
        int gpio = 4;  // Default GPIO
    #endif
    
    ESP_LOGI(LED_TAG, "Initializing LED controller: GPIO=%d", gpio);
    
    #ifdef CONFIG_LED_BRIGHTNESS
        uint8_t brightness = CONFIG_LED_BRIGHTNESS;
    #else
        uint8_t brightness = LED_DEFAULT_BRIGHTNESS;
    #endif
    
    if (!controller.init(gpio, brightness)) {
        ESP_LOGE(LED_TAG, "Failed to initialize LED controller");
        vTaskDelete(nullptr);
        return;
    }
    
    #ifdef CONFIG_LED_DEMO_TIMEOUT
        controller.setDemoTimeout(CONFIG_LED_DEMO_TIMEOUT * 1000);
    #endif
    
    TickType_t lastWake = xTaskGetTickCount();
    
    #ifdef CONFIG_LED_FPS
        const TickType_t frameDelay = pdMS_TO_TICKS(1000 / CONFIG_LED_FPS);
    #else
        const TickType_t frameDelay = pdMS_TO_TICKS(1000 / 30);  // 30 FPS default
    #endif
    
    bool lastBeat = false;
    LedAudioReadings readings;
    
    while (ledTaskRunning) {
        // Check if in OTA mode - render progress bar instead of effects
        if (controller.isOtaMode()) {
            controller.renderOtaProgress();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Get audio data from DSP processor using helper to avoid ICE
        readDspData(readings);
        
        // Detect beat edge
        bool beat = false;
        bool currentBeat = ledBeatDetected;
        if (currentBeat && !lastBeat) {
            beat = true;
        }
        lastBeat = currentBeat;
        
        // Calculate beat intensity from bass
        float beatIntensity = readings.bass;
        
        // Update LED controller
        controller.update(readings.bass, readings.mid, readings.high, 
                          readings.bassDb, readings.midDb, readings.highDb, 
                          beat, beatIntensity, readings.audioPlaying);
        
        vTaskDelayUntil(&lastWake, frameDelay);
    }
    
    ESP_LOGI(LED_TAG, "LED task stopped");
    vTaskDelete(nullptr);
}

inline void startLedTask(DSPProcessor* dsp, int priority = 3, int stackSize = 4096) {
    ESP_LOGI(LED_TAG, "startLedTask() called: dsp=%p, priority=%d, stack=%d", dsp, priority, stackSize);
    ESP_LOGI(LED_TAG, "Free heap: %lu, internal: %lu, PSRAM: %lu", 
             esp_get_free_heap_size(), 
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (!ledTaskRunning) {
        g_ledDsp = dsp;
        ledTaskRunning = true;
        
        // RMT driver requires task stack in internal RAM (not PSRAM)
        // Using standard xTaskCreatePinnedToCore for compatibility
        ESP_LOGI(LED_TAG, "Creating LED task with internal RAM stack...");
        BaseType_t ret = xTaskCreatePinnedToCore(
            ledTask, 
            "led_task", 
            stackSize, 
            nullptr, 
            priority, 
            &ledTaskHandle, 
            0  // Core 0 - core 1 is busy with Bluetooth
        );
        
        ESP_LOGI(LED_TAG, "xTaskCreatePinnedToCore returned: %d, handle=%p", ret, ledTaskHandle);
        if (ret != pdPASS) {
            ESP_LOGE(LED_TAG, "FAILED to create LED task! Error: %d", ret);
            ledTaskRunning = false;
        }
    } else {
        ESP_LOGW(LED_TAG, "LED task already running!");
    }
}

inline void stopLedTask() {
    ledTaskRunning = false;
    if (ledTaskHandle) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ledTaskHandle = nullptr;
    }
}
