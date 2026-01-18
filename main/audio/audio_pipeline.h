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

// Callback type for checking if I2S write should be skipped
typedef bool (*ShouldSkipWriteCallback)();

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
        , m_skipWriteCallback(nullptr)
        , m_wasSkipping(false)
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

        // Calculate required memory
        size_t poolSize = sizeof(AudioBuf) * APP_AUDIO_POOL_COUNT;
        ESP_LOGI(TAG, "Allocating audio pool: %d buffers x %d bytes = %u KB total",
                 APP_AUDIO_POOL_COUNT, (int)sizeof(AudioBuf), (unsigned)(poolSize / 1024));
        
        // Log available memory
        ESP_LOGI(TAG, "Free heap: internal=%u KB, PSRAM=%u KB",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

        // Try PSRAM first (if available), then fall back to internal
        m_pool = (AudioBuf*)heap_caps_malloc(poolSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (m_pool) {
            ESP_LOGI(TAG, "Audio pool allocated in PSRAM");
        } else {
            ESP_LOGW(TAG, "PSRAM alloc failed, trying internal heap");
            m_pool = (AudioBuf*)heap_caps_malloc(poolSize, MALLOC_CAP_8BIT);
            if (m_pool) {
                ESP_LOGI(TAG, "Audio pool allocated in internal RAM");
            }
        }
        if (!m_pool) {
            ESP_LOGE(TAG, "Failed to allocate audio pool - try reducing CONFIG_AUDIO_POOL_COUNT or CONFIG_AUDIO_POOL_BUF_SIZE");
            return false;
        }

        // Allocate DSP output buffer in DMA-capable internal RAM for fast I2S writes
        // This reduces latency as DMA can access internal RAM without cache contention
        size_t dspSize = sizeof(int32_t) * APP_DSP_OUT_FRAMES * 2;
        m_dspOut = (int32_t*)heap_caps_malloc(dspSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!m_dspOut) {
            ESP_LOGW(TAG, "DMA-capable RAM dsp_out failed, trying internal 8BIT");
            m_dspOut = (int32_t*)heap_caps_malloc(dspSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (!m_dspOut) {
            ESP_LOGW(TAG, "Internal RAM dsp_out failed, trying any available memory");
            m_dspOut = (int32_t*)heap_caps_malloc(dspSize, MALLOC_CAP_8BIT);
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

        ESP_LOGI(TAG, "Audio pipeline initialized: %d buffers @ %d bytes (total %u KB)", 
                 APP_AUDIO_POOL_COUNT, APP_AUDIO_POOL_BUF_SIZE, (unsigned)(poolSize / 1024));
        return true;
    }

    // Set callback to check if I2S writes should be skipped (e.g., during sound playback)
    void setSkipWriteCallback(ShouldSkipWriteCallback cb) {
        m_skipWriteCallback = cb;
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
        
        // Check if we should skip I2S write (e.g., sound effect playing)
        bool skipWrite = m_skipWriteCallback && m_skipWriteCallback();
        
        // Detect transition to exclusive sound - clear DMA for instant cutoff
        if (skipWrite && !m_wasSkipping) {
            i2s.zeroDMA();  // Clear I2S DMA for instant audio cutoff
        }
        m_wasSkipping = skipWrite;
        
        // Use longer timeout (20ms) to reduce underrun during codec transitions
        // Short timeout causes unnecessary returns when BT decode is briefly delayed
        if (xQueueReceive(m_audioQueue, &buf, pdMS_TO_TICKS(20)) != pdTRUE || !buf) {
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

            if (!skipWrite) {
                size_t bytesToWrite = frames * 2u * sizeof(int32_t);
                size_t written = i2s.write(m_dspOut, bytesToWrite);

                m_lastProcessMs = millis32();

                if (written < bytesToWrite) {
                    m_shortWriteCount++;
                }
            }
        }

        // Return buffer to free pool
        xQueueSend(m_freeQueue, &buf, 0);
    }

    // Clear all queued audio (fast path - no waiting)
    void clear() {
        if (!m_audioQueue || !m_freeQueue) return;
        
        AudioBuf *buf = nullptr;
        // Drain all buffers from audio queue back to free pool
        while (xQueueReceive(m_audioQueue, &buf, 0) == pdTRUE) {
            if (buf) {
                xQueueSend(m_freeQueue, &buf, 0);
            }
        }
    }

    // Aggressive clear - also zeros I2S DMA buffers for faster audio cutoff
    void clearWithDMA(I2SOutput& i2s) {
        clear();
        i2s.zeroDMA();
    }

    uint32_t getLastProcessMs() const { return m_lastProcessMs; }
    
    // Diagnostic: get buffer statistics
    uint32_t getDropCount() const { return m_dropCount; }
    uint32_t getEnqueueFailCount() const { return m_enqueueFail; }
    uint32_t getShortWriteCount() const { return m_shortWriteCount; }
    
    // Get queue fill level (0-100%)
    uint8_t getQueueFillPercent() const {
        if (!m_audioQueue) return 0;
        UBaseType_t waiting = uxQueueMessagesWaiting(m_audioQueue);
        return (uint8_t)((waiting * 100) / APP_AUDIO_POOL_COUNT);
    }

private:
    static constexpr const char* TAG = "AudioPipe";

    static uint32_t millis32() {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    void process16bit(AudioBuf *buf, uint32_t frames, uint8_t channels, DSPProcessor &dsp) {
        const int16_t *smp = (const int16_t *)buf->data;
        constexpr float scale16 = 1.0f / 32768.0f;
        constexpr float scaleOut = 2147483647.0f;
        
        // Process in blocks of 4 samples to reduce loop overhead
        uint32_t i = 0;
        const uint32_t unrollEnd = frames & ~3u;  // Round down to multiple of 4
        
        if (channels == 1) {
            // Mono path - unrolled
            for (; i < unrollEnd; i += 4) {
                float L0 = (float)smp[i] * scale16;
                float L1 = (float)smp[i + 1] * scale16;
                float L2 = (float)smp[i + 2] * scale16;
                float L3 = (float)smp[i + 3] * scale16;
                float R0 = L0, R1 = L1, R2 = L2, R3 = L3;
                
                dsp.processStereo(L0, R0);
                dsp.processStereo(L1, R1);
                dsp.processStereo(L2, R2);
                dsp.processStereo(L3, R3);
                
                m_dspOut[2 * i + 0] = (int32_t)(L0 * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R0 * scaleOut);
                m_dspOut[2 * (i + 1) + 0] = (int32_t)(L1 * scaleOut);
                m_dspOut[2 * (i + 1) + 1] = (int32_t)(R1 * scaleOut);
                m_dspOut[2 * (i + 2) + 0] = (int32_t)(L2 * scaleOut);
                m_dspOut[2 * (i + 2) + 1] = (int32_t)(R2 * scaleOut);
                m_dspOut[2 * (i + 3) + 0] = (int32_t)(L3 * scaleOut);
                m_dspOut[2 * (i + 3) + 1] = (int32_t)(R3 * scaleOut);
            }
            // Handle remaining samples
            for (; i < frames; i++) {
                float L = (float)smp[i] * scale16;
                float R = L;
                dsp.processStereo(L, R);
                m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
            }
        } else {
            // Stereo path - unrolled
            for (; i < unrollEnd; i += 4) {
                float L0 = (float)smp[(i) * 2 + 0] * scale16;
                float R0 = (float)smp[(i) * 2 + 1] * scale16;
                float L1 = (float)smp[(i + 1) * 2 + 0] * scale16;
                float R1 = (float)smp[(i + 1) * 2 + 1] * scale16;
                float L2 = (float)smp[(i + 2) * 2 + 0] * scale16;
                float R2 = (float)smp[(i + 2) * 2 + 1] * scale16;
                float L3 = (float)smp[(i + 3) * 2 + 0] * scale16;
                float R3 = (float)smp[(i + 3) * 2 + 1] * scale16;
                
                dsp.processStereo(L0, R0);
                dsp.processStereo(L1, R1);
                dsp.processStereo(L2, R2);
                dsp.processStereo(L3, R3);
                
                m_dspOut[2 * i + 0] = (int32_t)(L0 * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R0 * scaleOut);
                m_dspOut[2 * (i + 1) + 0] = (int32_t)(L1 * scaleOut);
                m_dspOut[2 * (i + 1) + 1] = (int32_t)(R1 * scaleOut);
                m_dspOut[2 * (i + 2) + 0] = (int32_t)(L2 * scaleOut);
                m_dspOut[2 * (i + 2) + 1] = (int32_t)(R2 * scaleOut);
                m_dspOut[2 * (i + 3) + 0] = (int32_t)(L3 * scaleOut);
                m_dspOut[2 * (i + 3) + 1] = (int32_t)(R3 * scaleOut);
            }
            // Handle remaining samples
            for (; i < frames; i++) {
                float L = (float)smp[i * channels + 0] * scale16;
                float R = (float)smp[i * channels + 1] * scale16;
                dsp.processStereo(L, R);
                m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
            }
        }
    }

    void process32bit(AudioBuf *buf, uint32_t frames, uint8_t channels, DSPProcessor &dsp) {
        const int32_t *smp = (const int32_t *)buf->data;
        constexpr float scale32 = 1.0f / 2147483648.0f;
        constexpr float scaleOut = 2147483647.0f;
        
        // Process in blocks of 4 samples to reduce loop overhead
        uint32_t i = 0;
        const uint32_t unrollEnd = frames & ~3u;  // Round down to multiple of 4
        
        if (channels == 1) {
            // Mono path - unrolled
            for (; i < unrollEnd; i += 4) {
                float L0 = (float)smp[i] * scale32;
                float L1 = (float)smp[i + 1] * scale32;
                float L2 = (float)smp[i + 2] * scale32;
                float L3 = (float)smp[i + 3] * scale32;
                float R0 = L0, R1 = L1, R2 = L2, R3 = L3;
                
                dsp.processStereo(L0, R0);
                dsp.processStereo(L1, R1);
                dsp.processStereo(L2, R2);
                dsp.processStereo(L3, R3);
                
                m_dspOut[2 * i + 0] = (int32_t)(L0 * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R0 * scaleOut);
                m_dspOut[2 * (i + 1) + 0] = (int32_t)(L1 * scaleOut);
                m_dspOut[2 * (i + 1) + 1] = (int32_t)(R1 * scaleOut);
                m_dspOut[2 * (i + 2) + 0] = (int32_t)(L2 * scaleOut);
                m_dspOut[2 * (i + 2) + 1] = (int32_t)(R2 * scaleOut);
                m_dspOut[2 * (i + 3) + 0] = (int32_t)(L3 * scaleOut);
                m_dspOut[2 * (i + 3) + 1] = (int32_t)(R3 * scaleOut);
            }
            // Handle remaining samples
            for (; i < frames; i++) {
                float L = (float)smp[i] * scale32;
                float R = L;
                dsp.processStereo(L, R);
                m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
            }
        } else {
            // Stereo path - unrolled
            for (; i < unrollEnd; i += 4) {
                float L0 = (float)smp[(i) * 2 + 0] * scale32;
                float R0 = (float)smp[(i) * 2 + 1] * scale32;
                float L1 = (float)smp[(i + 1) * 2 + 0] * scale32;
                float R1 = (float)smp[(i + 1) * 2 + 1] * scale32;
                float L2 = (float)smp[(i + 2) * 2 + 0] * scale32;
                float R2 = (float)smp[(i + 2) * 2 + 1] * scale32;
                float L3 = (float)smp[(i + 3) * 2 + 0] * scale32;
                float R3 = (float)smp[(i + 3) * 2 + 1] * scale32;
                
                dsp.processStereo(L0, R0);
                dsp.processStereo(L1, R1);
                dsp.processStereo(L2, R2);
                dsp.processStereo(L3, R3);
                
                m_dspOut[2 * i + 0] = (int32_t)(L0 * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R0 * scaleOut);
                m_dspOut[2 * (i + 1) + 0] = (int32_t)(L1 * scaleOut);
                m_dspOut[2 * (i + 1) + 1] = (int32_t)(R1 * scaleOut);
                m_dspOut[2 * (i + 2) + 0] = (int32_t)(L2 * scaleOut);
                m_dspOut[2 * (i + 2) + 1] = (int32_t)(R2 * scaleOut);
                m_dspOut[2 * (i + 3) + 0] = (int32_t)(L3 * scaleOut);
                m_dspOut[2 * (i + 3) + 1] = (int32_t)(R3 * scaleOut);
            }
            // Handle remaining samples
            for (; i < frames; i++) {
                float L = (float)smp[i * channels + 0] * scale32;
                float R = (float)smp[i * channels + 1] * scale32;
                dsp.processStereo(L, R);
                m_dspOut[2 * i + 0] = (int32_t)(L * scaleOut);
                m_dspOut[2 * i + 1] = (int32_t)(R * scaleOut);
            }
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
    
    ShouldSkipWriteCallback m_skipWriteCallback;
    bool m_wasSkipping;  // Track state transition for DMA clearing
};
