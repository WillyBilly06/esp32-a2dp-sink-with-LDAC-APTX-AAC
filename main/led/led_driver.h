#pragma once

// -----------------------------------------------------------
// WS2812B LED Driver using RMT (new driver API)
// Based on ESP-IDF official example
// Optimized for 16x16 matrix (256 LEDs)
// 
// Key features for BT+BLE+FFT stability:
// - Mutex-protected show() prevents overlapping transmissions
// - Single-frame transmission (no chunking)
// - DMA disabled (ESP32 original doesn't support RMT DMA)
// - 64-symbol RMT buffer with encoder refill via ISR
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "led_strip_encoder.h"
#include "led_config.h"

// RMT configuration
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000  // 10MHz resolution, 1 tick = 0.1us

// TX buffer size - send all LEDs in one transmission for reliability
#define LED_TX_BUFFER_SIZE (LED_MATRIX_COUNT * 3)  // 768 bytes for 256 LEDs

// RGB color structure
struct RGB {
    uint8_t r, g, b;
    
    RGB() : r(0), g(0), b(0) {}
    RGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    
    // Create from HSV (h: 0-255, s: 0-255, v: 0-255)
    static RGB fromHSV(uint8_t h, uint8_t s, uint8_t v) {
        if (s == 0) return RGB(v, v, v);
        
        uint8_t region = h / 43;
        uint8_t remainder = (h - (region * 43)) * 6;
        
        uint8_t p = (v * (255 - s)) >> 8;
        uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
        
        switch (region) {
            case 0: return RGB(v, t, p);
            case 1: return RGB(q, v, p);
            case 2: return RGB(p, v, t);
            case 3: return RGB(p, q, v);
            case 4: return RGB(t, p, v);
            default: return RGB(v, p, q);
        }
    }
    
    // Blend towards another color
    RGB blend(const RGB& other, uint8_t amount) const {
        return RGB(
            r + ((other.r - r) * amount >> 8),
            g + ((other.g - g) * amount >> 8),
            b + ((other.b - b) * amount >> 8)
        );
    }
    
    // Scale brightness
    RGB scale(uint8_t brightness) const {
        return RGB(
            (r * brightness) >> 8,
            (g * brightness) >> 8,
            (b * brightness) >> 8
        );
    }
    
    // Add with saturation
    RGB add(const RGB& other) const {
        uint16_t nr = r + other.r;
        uint16_t ng = g + other.g;
        uint16_t nb = b + other.b;
        return RGB(
            nr > 255 ? 255 : nr,
            ng > 255 ? 255 : ng,
            nb > 255 ? 255 : nb
        );
    }
};

// Common colors
namespace Colors {
    const RGB Black(0, 0, 0);
    const RGB White(255, 255, 255);
    const RGB Red(255, 0, 0);
    const RGB Green(0, 255, 0);
    const RGB Blue(0, 0, 255);
    const RGB Yellow(255, 255, 0);
    const RGB Cyan(0, 255, 255);
    const RGB Magenta(255, 0, 255);
    const RGB Orange(255, 128, 0);
    const RGB Purple(128, 0, 255);
    const RGB Pink(255, 64, 128);
}

class LedDriver {
public:
    LedDriver() : m_initialized(false), m_brightness(LED_DEFAULT_BRIGHTNESS), 
                  m_gpioPin(LED_DATA_PIN), m_rmtChannel(nullptr), m_ledEncoder(nullptr),
                  m_txMutex(nullptr), m_txInProgress(false) {
        // Initialize buffer to all black
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_buffer[i] = RGB(0, 0, 0);
        }
        // Create mutex for thread-safe refresh
        m_txMutex = xSemaphoreCreateMutex();
    }
    
    ~LedDriver() {
        if (m_ledEncoder) {
            rmt_del_encoder(m_ledEncoder);
            m_ledEncoder = nullptr;
        }
        if (m_rmtChannel) {
            rmt_disable(m_rmtChannel);
            rmt_del_channel(m_rmtChannel);
            m_rmtChannel = nullptr;
        }
        if (m_txMutex) {
            vSemaphoreDelete(m_txMutex);
            m_txMutex = nullptr;
        }
    }
    
    bool init(int gpioPin) {
        m_gpioPin = gpioPin;
        
        ESP_LOGI("LED", "Using internal RAM framebuffer (%d bytes)", (int)(LED_MATRIX_COUNT * sizeof(RGB)));
        ESP_LOGI("LED", "Creating RMT TX channel on GPIO %d", m_gpioPin);
        
        // Configure GPIO for output first
        gpio_config_t gpio_cfg = {};
        gpio_cfg.pin_bit_mask = (1ULL << m_gpioPin);
        gpio_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_cfg.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&gpio_cfg);
        if (err != ESP_OK) {
            ESP_LOGW("LED", "GPIO config failed (may be fine): %s", esp_err_to_name(err));
        }
        
        // Create RMT TX channel
        // ESP32 has 8 channels with 64 symbols each = 512 total
        // Use minimum 64 symbols (1 block) to reduce memory pressure
        // The encoder will refill the buffer via ISR as data is transmitted
        rmt_tx_channel_config_t tx_chan_config = {};
        tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_chan_config.gpio_num = (gpio_num_t)m_gpioPin;
        tx_chan_config.mem_block_symbols = 64;   // Minimum single block
        tx_chan_config.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
        tx_chan_config.trans_queue_depth = 1;    // Minimal queue to reduce memory
        tx_chan_config.flags.invert_out = false;
        tx_chan_config.flags.with_dma = false;   // ESP32 original doesn't support DMA for RMT
        
        err = rmt_new_tx_channel(&tx_chan_config, &m_rmtChannel);
        if (err != ESP_OK) {
            ESP_LOGE("LED", "Failed to create RMT TX channel: %s", esp_err_to_name(err));
            return false;
        }
        
        ESP_LOGI("LED", "Installing LED strip encoder");
        
        // Install LED strip encoder
        led_strip_encoder_config_t encoder_config = {};
        encoder_config.resolution = RMT_LED_STRIP_RESOLUTION_HZ;
        
        err = rmt_new_led_strip_encoder(&encoder_config, &m_ledEncoder);
        if (err != ESP_OK) {
            ESP_LOGE("LED", "Failed to create LED strip encoder: %s", esp_err_to_name(err));
            rmt_del_channel(m_rmtChannel);
            m_rmtChannel = nullptr;
            return false;
        }
        
        ESP_LOGI("LED", "Enabling RMT TX channel");
        
        // Enable RMT TX channel
        err = rmt_enable(m_rmtChannel);
        if (err != ESP_OK) {
            ESP_LOGE("LED", "Failed to enable RMT channel: %s", esp_err_to_name(err));
            rmt_del_encoder(m_ledEncoder);
            rmt_del_channel(m_rmtChannel);
            m_ledEncoder = nullptr;
            m_rmtChannel = nullptr;
            return false;
        }
        
        m_initialized = true;
        
        // Test pattern: light up first 4 LEDs in R,G,B,W
        clear();
        m_buffer[0] = RGB(255, 0, 0);    // Red
        m_buffer[1] = RGB(0, 255, 0);    // Green  
        m_buffer[2] = RGB(0, 0, 255);    // Blue
        m_buffer[3] = RGB(255, 255, 255); // White
        show();
        ESP_LOGI("LED", "Test pattern sent - first 4 LEDs should be R,G,B,W");
        
        vTaskDelay(pdMS_TO_TICKS(500));  // Show test pattern briefly
        
        clear();
        show();
        
        ESP_LOGI("LED", "WS2812B driver initialized (RMT): %dx%d matrix on GPIO%d (brightness=%d)", 
                 LED_MATRIX_WIDTH, LED_MATRIX_HEIGHT, m_gpioPin, m_brightness);
        return true;
    }
    
    // Set global brightness (0-255)
    void setBrightness(uint8_t brightness) {
        m_brightness = brightness;
    }
    
    uint8_t getBrightness() const { return m_brightness; }
    
    // Clear all LEDs
    void clear() {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_buffer[i] = RGB(0, 0, 0);
        }
    }
    
    // Set pixel by linear index
    void setPixel(int index, const RGB& color) {
        if (index >= 0 && index < LED_MATRIX_COUNT) {
            m_buffer[index] = color;
        }
    }
    
    // Set pixel by X,Y coordinates (handles serpentine layout)
    void setPixelXY(int x, int y, const RGB& color) {
        if (x < 0 || x >= LED_MATRIX_WIDTH || y < 0 || y >= LED_MATRIX_HEIGHT) return;
        
        // Serpentine layout: odd rows are reversed
        int index;
        if (y & 1) {
            index = y * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - x);
        } else {
            index = y * LED_MATRIX_WIDTH + x;
        }
        m_buffer[index] = color;
    }
    
    // Get pixel
    RGB getPixel(int index) const {
        if (index >= 0 && index < LED_MATRIX_COUNT) {
            return m_buffer[index];
        }
        return Colors::Black;
    }
    
    RGB getPixelXY(int x, int y) const {
        if (x < 0 || x >= LED_MATRIX_WIDTH || y < 0 || y >= LED_MATRIX_HEIGHT) {
            return Colors::Black;
        }
        int index;
        if (y & 1) {
            index = y * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - x);
        } else {
            index = y * LED_MATRIX_WIDTH + x;
        }
        return m_buffer[index];
    }
    
    // Fill entire matrix with color
    void fill(const RGB& color) {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_buffer[i] = color;
        }
    }
    
    // Fill rectangle
    void fillRect(int x, int y, int w, int h, const RGB& color) {
        for (int dy = 0; dy < h; dy++) {
            for (int dx = 0; dx < w; dx++) {
                setPixelXY(x + dx, y + dy, color);
            }
        }
    }
    
    // Draw horizontal line
    void drawHLine(int x, int y, int len, const RGB& color) {
        for (int i = 0; i < len; i++) {
            setPixelXY(x + i, y, color);
        }
    }
    
    // Draw vertical line
    void drawVLine(int x, int y, int len, const RGB& color) {
        for (int i = 0; i < len; i++) {
            setPixelXY(x, y + i, color);
        }
    }
    
    // Fade all pixels by amount (0-255, 255 = no fade)
    void fadeAll(uint8_t scale) {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_buffer[i].r = (m_buffer[i].r * scale) >> 8;
            m_buffer[i].g = (m_buffer[i].g * scale) >> 8;
            m_buffer[i].b = (m_buffer[i].b * scale) >> 8;
        }
    }
    
    // Blur effect (simple box blur)
    void blur(uint8_t amount) {
        RGB temp[LED_MATRIX_COUNT];
        memcpy(temp, m_buffer, sizeof(temp));
        
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                int r = 0, g = 0, b = 0, count = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < LED_MATRIX_WIDTH && ny >= 0 && ny < LED_MATRIX_HEIGHT) {
                            int idx = (ny & 1) ? ny * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - nx) 
                                               : ny * LED_MATRIX_WIDTH + nx;
                            r += temp[idx].r;
                            g += temp[idx].g;
                            b += temp[idx].b;
                            count++;
                        }
                    }
                }
                RGB blurred(r / count, g / count, b / count);
                RGB original = getPixelXY(x, y);
                setPixelXY(x, y, original.blend(blurred, amount));
            }
        }
    }
    
    // Send buffer to LEDs using RMT
    // MUTEX PROTECTED - prevents overlapping transmissions which cause RMT stalls
    esp_err_t show() {
        if (!m_initialized || !m_rmtChannel || !m_ledEncoder) {
            return ESP_ERR_INVALID_STATE;
        }
        
        // Try to take mutex - skip frame if another transmission is in progress
        if (xSemaphoreTake(m_txMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            // Another transmission in progress - skip this frame
            return ESP_ERR_TIMEOUT;
        }
        
        // TX buffer in internal RAM (static to avoid stack allocation)
        static uint8_t txBuffer[LED_TX_BUFFER_SIZE];
        
        // Generate GRB data from framebuffer with brightness applied
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            RGB scaled = m_buffer[i].scale(m_brightness);
#if LED_COLOR_ORDER_GRB
            txBuffer[i * 3 + 0] = scaled.g;
            txBuffer[i * 3 + 1] = scaled.r;
            txBuffer[i * 3 + 2] = scaled.b;
#else
            txBuffer[i * 3 + 0] = scaled.r;
            txBuffer[i * 3 + 1] = scaled.g;
            txBuffer[i * 3 + 2] = scaled.b;
#endif
        }
        
        // Reset encoder before transmission
        rmt_encoder_reset(m_ledEncoder);
        
        // Transmit entire frame
        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;
        
        esp_err_t err = rmt_transmit(m_rmtChannel, m_ledEncoder, txBuffer, sizeof(txBuffer), &tx_config);
        if (err != ESP_OK) {
            xSemaphoreGive(m_txMutex);
            return err;
        }
        
        // Wait for transmission: 256 LEDs × 24 bits × 1.25us ≈ 8ms
        // Use 100ms timeout for safety margin
        err = rmt_tx_wait_all_done(m_rmtChannel, pdMS_TO_TICKS(100));
        
        if (err == ESP_ERR_TIMEOUT) {
            static uint32_t s_lastLogTime = 0;
            static int s_errorCount = 0;
            s_errorCount++;
            
            uint32_t now = xTaskGetTickCount();
            if (now - s_lastLogTime > pdMS_TO_TICKS(5000)) {
                ESP_LOGW("LED", "RMT timeout (count=%d) - GPIO%d check wiring", 
                         s_errorCount, m_gpioPin);
                s_lastLogTime = now;
            }
            
            // Recovery: disable and re-enable RMT
            rmt_disable(m_rmtChannel);
            rmt_encoder_reset(m_ledEncoder);
            rmt_enable(m_rmtChannel);
        }
        
        xSemaphoreGive(m_txMutex);
        return err;
    }
    
private:
    bool m_initialized;
    uint8_t m_brightness;
    int m_gpioPin;
    rmt_channel_handle_t m_rmtChannel;
    rmt_encoder_handle_t m_ledEncoder;
    SemaphoreHandle_t m_txMutex;      // Mutex to prevent overlapping transmissions
    volatile bool m_txInProgress;     // Flag for transmission state
    RGB m_buffer[LED_MATRIX_COUNT];   // Fixed internal RAM framebuffer
};
