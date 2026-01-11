#pragma once

// -----------------------------------------------------------
// Audio Pipeline - manages audio buffer queue and DSP task
// Decouples BT audio decode from I2S output
// -----------------------------------------------------------

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "../config/app_config.h"
#include "../dsp/dsp_processor.h"
#include "i2s_output.h"

// Audio buffer structure
struct AudioBuf {
    uint32_t cap;
    uint32_t len;
    uint8_t bits;
    uint8_t channels;
    uint8_t reserved[2];
    uint8_t data[APP_AUDIO_POOL_BUF_SIZE];
};

class AudioPipeline {
public:
    AudioPipeline() 
        : m_audioQueue(nullptr)
        , m_freeQueue(nullptr)
        , m_pool(nullptr)
        , m_dspOut(nullptr)
        , m_dropCount(0)
        , m_enqueueFail(0)
        , m_shortWriteCount(0)
        , m_lastProcessMs(0)
    {
    }

    ~AudioPipeline() {
        if (m_pool) heap_caps_free(m_pool);
        if (m_dspOut) heap_caps_free(m_dspOut);
    }

    // Initialize queues and buffer pool
    bool init() {
        m_audioQueue = xQueueCreate(APP_AUDIO_POOL_COUNT, sizeof(void*));
        m_freeQueue = xQueueCreate(APP_AUDIO_POOL_COUNT, sizeof(void*));
        
        if (!m_audioQueue || !m_freeQueue) {
            ESP_LOGE(TAG, "Failed to create audio queues");
            return false;
        }

        // Allocate pool in SPIRAM
        m_pool = (AudioBuf*)heap_caps_malloc(sizeof(AudioBuf) * APP_AUDIO_POOL_COUNT, 
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!m_pool) {
            ESP_LOGW(TAG, "SPIRAM alloc failed, trying internal heap");
            m_pool = (AudioBuf*)heap_caps_malloc(sizeof(AudioBuf) * APP_AUDIO_POOL_COUNT, 
                                                  MALLOC_CAP_8BIT);
        }
        if (!m_pool) {
            ESP_LOGE(TAG, "Failed to allocate audio pool");
            return false;
        }

        // Allocate DSP output buffer (prefer internal RAM for low latency)
        m_dspOut = (int32_t*)heap_caps_malloc(sizeof(int32_t) * APP_DSP_OUT_FRAMES * 2, 
                                               MALLOC_CAP_INTERNAL);
        if (!m_dspOut) {
            ESP_LOGW(TAG, "Internal RAM dsp_out failed, using SPIRAM");
            m_dspOut = (int32_t*)heap_caps_malloc(sizeof(int32_t) * APP_DSP_OUT_FRAMES * 2, 
                                                   MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        }
        if (!m_dspOut) {
            ESP_LOGE(TAG, "Failed to allocate DSP output buffer");
            return false;
        }

        // Initialize pool and push to free queue
        for (int i = 0; i < APP_AUDIO_POOL_COUNT; i++) {
            m_pool[i].cap = APP_AUDIO_POOL_BUF_SIZE;
            m_pool[i].len = 0;
            m_pool[i].bits = 16;
            m_pool[i].channels = 2;
            void *p = &m_pool[i];
            xQueueSend(m_freeQueue, &p, 0);
        }

        ESP_LOGI(TAG, "Audio pipeline initialized: %d buffers @ %d bytes", 
                 APP_AUDIO_POOL_COUNT, APP_AUDIO_POOL_BUF_SIZE);
        return true;
    }

    // Enqueue audio data from BT callback (non-blocking)
    void enqueue(const uint8_t *data, uint32_t len, uint8_t bits, uint8_t channels) {
        if (!m_audioQueue || !m_freeQueue || len == 0) return;

        size_t remaining = len;
        const uint8_t *ptr = data;

        while (remaining > 0) {
            AudioBuf *buf = nullptr;
            if (xQueueReceive(m_freeQueue, &buf, 0) != pdTRUE) {
                m_dropCount++;
                if ((m_dropCount % 500) == 0) {
                    ESP_LOGW(TAG, "Buffer drop count: %u", (unsigned)m_dropCount);
                }
                break;
            }

            size_t copyLen = (remaining > buf->cap) ? buf->cap : remaining;
            buf->len = copyLen;
            buf->bits = bits;
            buf->channels = channels;
            memcpy(buf->data, ptr, copyLen);

            if (xQueueSend(m_audioQueue, &buf, 0) != pdTRUE) {
                m_enqueueFail++;
                xQueueSend(m_freeQueue, &buf, 0);
                break;
            }

            ptr += copyLen;
            remaining -= copyLen;
        }
    }

    // Process queued audio (called from TX task)
    void processBuffer(DSPProcessor &dsp, I2SOutput &i2s) {
        AudioBuf *buf = nullptr;
        
        if (xQueueReceive(m_audioQueue, &buf, pdMS_TO_TICKS(10)) != pdTRUE || !buf) {
            return;
        }

        uint32_t len = buf->len;
        uint8_t bits = buf->bits;
        uint8_t channels = buf->channels ? buf->channels : 2;

        uint32_t bytesPerSample = (bits <= 16) ? 2u : 4u;
        uint32_t bytesPerFrame = bytesPerSample * channels;
        
        uint32_t frames;
        if (bytesPerFrame == 0) {
            frames = 0;
        } else if (bytesPerFrame == 4) {
            frames = len >> 2;
        } else if (bytesPerFrame == 8) {
            frames = len >> 3;
        } else {
            frames = len / bytesPerFrame;
        }

        if (frames > 0) {
            if (frames > APP_DSP_OUT_FRAMES) frames = APP_DSP_OUT_FRAMES;

            if (bytesPerSample == 2) {
                process16bit(buf, frames, channels, dsp);
            } else {
                process32bit(buf, frames, channels, dsp);
            }

            size_t bytesToWrite = frames * 2u * sizeof(int32_t);
            size_t written = i2s.write(m_dspOut, bytesToWrite);

            m_lastProcessMs = millis32();

            if (written < bytesToWrite) {
                m_shortWriteCount++;
            }
        }

        // Return buffer to free pool
        xQueueSend(m_freeQueue, &buf, 0);
    }

    // Clear all queued audio
    void clear() {
        if (!m_audioQueue || !m_freeQueue) return;
        
        AudioBuf *buf = nullptr;
        while (xQueueReceive(m_audioQueue, &buf, 0) == pdTRUE) {
            if (buf) {
                xQueueSend(m_freeQueue, &buf, 0);
            }
        }
    }

    uint32_t getLastProcessMs() const { return m_lastProcessMs; }

private:
    static constexpr const char* TAG = "AudioPipe";

    static uint32_t millis32() {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    void process16bit(AudioBuf *buf, uint32_t frames, uint8_t channels, DSPProcessor &dsp) {
        const int16_t *smp = (const int16_t *)buf->data;
        constexpr float scale16 = 1.0f / 32768.0f;
        constexpr float scaleOut = 2147483647.0f;
        
        for (uint32_t i = 0; i < frames; i++) {
            int16_t l_raw = 0, r_raw = 0;
            if (channels == 1) {
                l_raw = smp[i];
                r_raw = l_raw;
            } else {
                l_raw = smp[i * channels + 0];
                r_raw = smp[i * channels + 1];
            }

            float L = (float)l_raw * scale16;
            float R = (float)r_raw * scale16;

            dsp.processStereo(L, R);

            // Convert to 32-bit output (soft clipper already limits to [-1, 1])
            m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
            m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
        }
    }

    void process32bit(AudioBuf *buf, uint32_t frames, uint8_t channels, DSPProcessor &dsp) {
        const int32_t *smp = (const int32_t *)buf->data;
        constexpr float scale32 = 1.0f / 2147483648.0f;
        constexpr float scaleOut = 2147483647.0f;
        
        for (uint32_t i = 0; i < frames; i++) {
            int32_t l_raw = 0, r_raw = 0;
            if (channels == 1) {
                l_raw = smp[i];
                r_raw = l_raw;
            } else {
                l_raw = smp[i * channels + 0];
                r_raw = smp[i * channels + 1];
            }

            float L = (float)l_raw * scale32;
            float R = (float)r_raw * scale32;

            dsp.processStereo(L, R);

            // Convert to 32-bit output (soft clipper already limits to [-1, 1])
            m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
            m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
        }
    }

    QueueHandle_t m_audioQueue;
    QueueHandle_t m_freeQueue;
    AudioBuf *m_pool;
    int32_t *m_dspOut;

    volatile uint32_t m_dropCount;
    volatile uint32_t m_enqueueFail;
    volatile uint32_t m_shortWriteCount;
    volatile uint32_t m_lastProcessMs;
};
