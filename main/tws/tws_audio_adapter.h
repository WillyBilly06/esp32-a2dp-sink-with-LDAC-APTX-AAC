#pragma once

/*
 * tws_audio_adapter.h
 *
 * Glue between TWS and the audio pipeline.
 * Splits channels, adds sync delay, forwards audio to partner.
 */

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "../config/app_config.h"
#include "tws_espnow.h"

// Forward declarations
class DSPProcessor;
class I2SOutput;

// -----------------------------------------------------------
// TWS Audio Buffer for sync delay
// -----------------------------------------------------------
struct TwsSyncBuffer {
    int32_t* data;          // Ring buffer for delayed playback
    uint32_t capacity;      // Total capacity in samples (L+R interleaved)
    uint32_t writePos;      // Write position
    uint32_t readPos;       // Read position
    uint32_t fillLevel;     // Current fill level in samples
    uint32_t targetDelay;   // Target delay in samples
};

// -----------------------------------------------------------
// TWS Audio Adapter Class
// -----------------------------------------------------------
class TwsAudioAdapter {
public:
    static constexpr const char* TAG = "TwsAudio";
    
    TwsAudioAdapter()
        : m_syncBuf({nullptr, 0, 0, 0, 0, 0})
        , m_rxBuf(nullptr)
        , m_rxBufFrames(0)
        , m_sampleRate(44100)
        , m_initialized(false)
        , m_lastRxTime(0)
    {
    }

    ~TwsAudioAdapter() {
        deinit();
    }

    // Initialize the adapter based on TWS role
    bool init(uint32_t sampleRate = 44100) {
        m_sampleRate = sampleRate;
        
        TwsManager& tws = TwsManager::getInstance();
        if (!tws.isEnabled()) {
            ESP_LOGI(TAG, "TWS not enabled, adapter inactive");
            return true;
        }

        tws.setSampleRate(sampleRate);

        if (tws.isPrimary()) {
            // Primary: Create sync delay buffer
            // Buffer size = sync delay in samples (stereo, so *2)
            uint32_t delayMs = APP_TWS_SYNC_DELAY_MS;
            uint32_t delaySamples = (sampleRate * delayMs) / 1000;
            uint32_t bufferSize = delaySamples * 2 * sizeof(int32_t);  // Stereo int32
            
            // Add 50% headroom
            m_syncBuf.capacity = delaySamples * 3;
            m_syncBuf.targetDelay = delaySamples;
            
            bufferSize = m_syncBuf.capacity * 2 * sizeof(int32_t);
            m_syncBuf.data = (int32_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!m_syncBuf.data) {
                m_syncBuf.data = (int32_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT);
            }
            
            if (!m_syncBuf.data) {
                ESP_LOGE(TAG, "Failed to allocate sync buffer (%u bytes)", bufferSize);
                return false;
            }
            
            memset(m_syncBuf.data, 0, bufferSize);
            m_syncBuf.writePos = 0;
            m_syncBuf.readPos = 0;
            m_syncBuf.fillLevel = 0;
            
            ESP_LOGI(TAG, "Primary: Sync buffer initialized (%u ms, %u samples)", 
                     delayMs, delaySamples);
        } else {
            // Secondary: Create receive buffer
            // Larger buffer to handle network jitter
            uint32_t bufferMs = 100;  // 100ms buffer
            m_rxBufFrames = (sampleRate * bufferMs) / 1000;
            size_t bufferSize = m_rxBufFrames * sizeof(int16_t);  // Mono int16
            
            m_rxBuf = (int16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_8BIT);
            if (!m_rxBuf) {
                ESP_LOGE(TAG, "Failed to allocate RX buffer");
                return false;
            }
            
            memset(m_rxBuf, 0, bufferSize);
            
            ESP_LOGI(TAG, "Secondary: RX buffer initialized (%u ms, %u frames)",
                     bufferMs, m_rxBufFrames);
        }

        m_initialized = true;
        return true;
    }

    void deinit() {
        if (m_syncBuf.data) {
            heap_caps_free(m_syncBuf.data);
            m_syncBuf.data = nullptr;
        }
        if (m_rxBuf) {
            heap_caps_free(m_rxBuf);
            m_rxBuf = nullptr;
        }
        m_initialized = false;
    }

    // Update sample rate (called when codec changes)
    void setSampleRate(uint32_t rate) {
        if (rate != m_sampleRate) {
            m_sampleRate = rate;
            TwsManager::getInstance().setSampleRate(rate);
            
            // Reinitialize buffers for new sample rate
            if (m_initialized) {
                deinit();
                init(rate);
            }
        }
    }

    // Process audio for TWS (called from primary's audio pipeline)
    // Input: DSP-processed stereo int32 samples
    // Output: Modified samples for local playback (one channel), sends other to secondary
    void processForTws(int32_t* samples, uint32_t frames) {
        TwsManager& tws = TwsManager::getInstance();
        if (!tws.isEnabled() || !tws.isPrimary() || !m_initialized) return;
        
        // 1. Convert int32 to int16 for ESP-NOW (reduces bandwidth)
        // 2. Extract the channel to send to secondary
        // 3. Send via ESP-NOW
        // 4. Apply sync delay to local output
        // 5. Modify samples to play only local channel
        
        TwsChannel localChannel = tws.getLocalChannel();
        int localOffset = (localChannel == TwsChannel::Left) ? 0 : 1;
        int remoteOffset = (localChannel == TwsChannel::Left) ? 1 : 0;
        
        // Prepare int16 buffer for sending
        int16_t sendBuf[frames];
        for (uint32_t i = 0; i < frames; i++) {
            // Convert to 16-bit (shift down from 32-bit)
            sendBuf[i] = (int16_t)(samples[i * 2 + remoteOffset] >> 16);
        }
        
        // Send to secondary
        tws.sendAudio(sendBuf, frames, 16, 1);
        
        // Apply sync delay to local samples
        // Write current samples to sync buffer, read delayed samples
        if (m_syncBuf.data) {
            for (uint32_t i = 0; i < frames; i++) {
                // Write to sync buffer
                uint32_t writeIdx = m_syncBuf.writePos * 2;
                m_syncBuf.data[writeIdx + 0] = samples[i * 2 + 0];
                m_syncBuf.data[writeIdx + 1] = samples[i * 2 + 1];
                m_syncBuf.writePos = (m_syncBuf.writePos + 1) % m_syncBuf.capacity;
                
                if (m_syncBuf.fillLevel < m_syncBuf.capacity) {
                    m_syncBuf.fillLevel++;
                }
                
                // Read from sync buffer (if enough buffered)
                if (m_syncBuf.fillLevel >= m_syncBuf.targetDelay) {
                    uint32_t readIdx = m_syncBuf.readPos * 2;
                    
                    // Play only local channel on both L and R (mono from our channel)
                    int32_t localSample = m_syncBuf.data[readIdx + localOffset];
                    samples[i * 2 + 0] = localSample;
                    samples[i * 2 + 1] = localSample;
                    
                    m_syncBuf.readPos = (m_syncBuf.readPos + 1) % m_syncBuf.capacity;
                    m_syncBuf.fillLevel--;
                } else {
                    // Still filling buffer, output silence
                    samples[i * 2 + 0] = 0;
                    samples[i * 2 + 1] = 0;
                }
            }
        } else {
            // No sync buffer, just play local channel immediately
            for (uint32_t i = 0; i < frames; i++) {
                int32_t localSample = samples[i * 2 + localOffset];
                samples[i * 2 + 0] = localSample;
                samples[i * 2 + 1] = localSample;
            }
        }
    }

    // Receive and process audio (called from secondary's audio task)
    // Returns number of frames output to the provided buffer
    uint32_t receiveAndProcess(int32_t* outSamples, uint32_t maxFrames, DSPProcessor& dsp) {
        TwsManager& tws = TwsManager::getInstance();
        if (!tws.isEnabled() || !tws.isSecondary() || !m_initialized) return 0;
        
        // Receive audio from ESP-NOW
        int16_t rxBuf[256];  // Max frames per packet
        uint8_t bits = 16;
        uint32_t timestamp = 0;
        
        uint32_t frames = tws.receiveAudio(rxBuf, 256, &bits, &timestamp);
        if (frames == 0) return 0;
        
        m_lastRxTime = esp_timer_get_time();
        
        if (frames > maxFrames) frames = maxFrames;
        
        // Convert to int32 and expand mono to stereo (same sample on both channels)
        constexpr float scale16 = 1.0f / 32768.0f;
        constexpr float scaleOut = 2147483647.0f;
        
        for (uint32_t i = 0; i < frames; i++) {
            float sample = (float)rxBuf[i] * scale16;
            float L = sample, R = sample;
            
            // Apply DSP
            dsp.processStereo(L, R);
            
            outSamples[i * 2 + 0] = (int32_t)(L * scaleOut);
            outSamples[i * 2 + 1] = (int32_t)(R * scaleOut);
        }
        
        return frames;
    }

    // Check if secondary has active audio connection
    bool isReceiving() const {
        if (!m_initialized) return false;
        int64_t now = esp_timer_get_time();
        return (now - m_lastRxTime) < 500000;  // 500ms timeout
    }

    // Get sync buffer fill percentage
    uint8_t getSyncFillPercent() const {
        if (!m_syncBuf.data || m_syncBuf.targetDelay == 0) return 0;
        return (uint8_t)((m_syncBuf.fillLevel * 100) / m_syncBuf.targetDelay);
    }

    // Singleton
    static TwsAudioAdapter& getInstance() {
        static TwsAudioAdapter instance;
        return instance;
    }

private:
    TwsSyncBuffer m_syncBuf;
    int16_t* m_rxBuf;
    uint32_t m_rxBufFrames;
    uint32_t m_sampleRate;
    bool m_initialized;
    int64_t m_lastRxTime;
};
