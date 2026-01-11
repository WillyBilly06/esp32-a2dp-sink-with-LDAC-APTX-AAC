#pragma once

// -----------------------------------------------------------
// I2S Output - manages I2S driver for audio output
// Always operates in 32-bit stereo mode
// -----------------------------------------------------------

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "../config/app_config.h"

class I2SOutput {
public:
    I2SOutput() 
        : m_initialized(false)
        , m_sampleRate(0)
        , m_reconfig(false)
        , m_mutex(nullptr)
    {
    }

    ~I2SOutput() {
        if (m_mutex) {
            vSemaphoreDelete(m_mutex);
        }
    }

    // Initialize I2S driver (call once at startup)
    esp_err_t init(uint32_t sampleRate) {
        if (m_initialized) return ESP_OK;

        // Create mutex for thread-safe access
        m_mutex = xSemaphoreCreateMutex();
        if (!m_mutex) {
            ESP_LOGE(TAG, "Failed to create I2S mutex");
            return ESP_ERR_NO_MEM;
        }

        i2s_config_t i2s_config = {};
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.sample_rate = sampleRate;
        i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
        i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
        i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB);
        i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
        i2s_config.dma_buf_count = 12;
        i2s_config.dma_buf_len = 512;
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
        i2s_config.fixed_mclk = 0;

        auto install_with = [&](int dma_count, int dma_len) -> esp_err_t {
            i2s_config.dma_buf_count = dma_count;
            i2s_config.dma_buf_len = dma_len;
            esp_err_t e = i2s_driver_install((i2s_port_t)APP_I2S_PORT, &i2s_config, 0, NULL);
            if (e != ESP_OK) {
                ESP_LOGE(TAG, "i2s_driver_install failed (dma_count=%d dma_len=%d): %s", 
                         dma_count, dma_len, esp_err_to_name(e));
            }
            return e;
        };

        esp_err_t err = install_with(12, 512);
        if (err == ESP_ERR_NO_MEM) {
            err = install_with(8, 256);
        }
        if (err != ESP_OK) {
            return err;
        }

        i2s_pin_config_t pins = {};
        pins.mck_io_num = I2S_PIN_NO_CHANGE;
        pins.bck_io_num = APP_I2S_BCK_PIN;
        pins.ws_io_num = APP_I2S_LRCK_PIN;
        pins.data_out_num = APP_I2S_DATA_PIN;
        pins.data_in_num = I2S_PIN_NO_CHANGE;

        err = i2s_set_pin((i2s_port_t)APP_I2S_PORT, &pins);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(err));
            i2s_driver_uninstall((i2s_port_t)APP_I2S_PORT);
            return err;
        }

        err = i2s_set_clk((i2s_port_t)APP_I2S_PORT, sampleRate, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_set_clk failed: %s", esp_err_to_name(err));
            i2s_driver_uninstall((i2s_port_t)APP_I2S_PORT);
            return err;
        }

        m_initialized = true;
        m_sampleRate = sampleRate;
        ESP_LOGI(TAG, "I2S initialized: sr=%u, 32-bit stereo", (unsigned)sampleRate);
        return ESP_OK;
    }

    // Update sample rate (clock only, no driver reinstall)
    void updateClock(uint32_t sampleRate) {
        if (!m_initialized) return;
        if (sampleRate == 0) sampleRate = APP_I2S_DEFAULT_SR;
        if (m_sampleRate == sampleRate) return;

        lock();
        m_reconfig = true;

        i2s_stop((i2s_port_t)APP_I2S_PORT);
        i2s_zero_dma_buffer((i2s_port_t)APP_I2S_PORT);

        esp_err_t err = i2s_set_clk((i2s_port_t)APP_I2S_PORT, sampleRate, 
                                     I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
        if (err == ESP_OK) {
            m_sampleRate = sampleRate;
            ESP_LOGI(TAG, "I2S clock updated: sr=%u", (unsigned)sampleRate);
        } else {
            ESP_LOGE(TAG, "i2s_set_clk failed: %s", esp_err_to_name(err));
        }

        i2s_start((i2s_port_t)APP_I2S_PORT);
        m_reconfig = false;
        unlock();
    }

    // Reset to default sample rate (on disconnect)
    void resetToDefault() {
        updateClock(APP_I2S_DEFAULT_SR);
        zeroDMA();
    }

    // Write audio data
    size_t write(const void *data, size_t bytes) {
        if (!m_initialized || m_reconfig) return 0;
        
        size_t written = 0;
        lock();
        i2s_write((i2s_port_t)APP_I2S_PORT, data, bytes, &written, portMAX_DELAY);
        unlock();
        return written;
    }

    // Zero DMA buffer
    void zeroDMA() {
        if (m_initialized) {
            i2s_zero_dma_buffer((i2s_port_t)APP_I2S_PORT);
        }
    }

    // Stop I2S
    void stop() {
        if (m_initialized) {
            i2s_stop((i2s_port_t)APP_I2S_PORT);
        }
    }

    // Start I2S
    void start() {
        if (m_initialized) {
            i2s_start((i2s_port_t)APP_I2S_PORT);
        }
    }

    bool isInitialized() const { return m_initialized; }
    bool isReconfiguring() const { return m_reconfig; }
    uint32_t getSampleRate() const { return m_sampleRate; }

    void lock() {
        if (m_mutex) xSemaphoreTake(m_mutex, portMAX_DELAY);
    }

    void unlock() {
        if (m_mutex) xSemaphoreGive(m_mutex);
    }

private:
    static constexpr const char* TAG = "I2S";

    bool m_initialized;
    uint32_t m_sampleRate;
    volatile bool m_reconfig;
    SemaphoreHandle_t m_mutex;
};
