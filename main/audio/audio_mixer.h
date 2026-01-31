#pragma once

// -----------------------------------------------------------
// Audio Mixer - mixes BT audio with WAV overlays using ducking
// Single I2S output, software mixing with gain ramps
// -----------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "i2s_output.h"
#include "../dsp/fast_math.h"

// Ring buffer sizes (in PSRAM)
#define MIXER_RB_BT_BYTES    (48 * 1024)   // 48KB for BT audio
#define MIXER_RB_OVL_BYTES   (32 * 1024)   // 32KB for overlay audio

// Mixer chunk size (frames per mix cycle)
#define MIXER_CHUNK_FRAMES   256
#define MIXER_BYTES_PER_FRAME 8  // stereo 32-bit

// Q15 fixed point for gain (32768 = 1.0)
#define Q15_UNITY           32768
#define Q15_DUCK_DEFAULT    6554    // ~0.2 (duck BT to 20% while overlay plays)

// -----------------------------------------------------------
// Simple Ring Buffer (lockless single producer/single consumer)
// -----------------------------------------------------------
struct MixerRingBuf {
    uint8_t* buf;
    size_t size;
    volatile size_t readPos;
    volatile size_t writePos;
    
    void init(size_t bytes, bool preferPsram = true) {
        if (preferPsram) {
            buf = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (!buf) {
            buf = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        size = buf ? bytes : 0;
        readPos = writePos = 0;
    }
    
    void reset() {
        readPos = writePos = 0;
    }
    
    size_t available() const {
        size_t w = writePos;
        size_t r = readPos;
        return (w >= r) ? (w - r) : (size - r + w);
    }
    
    size_t freeSpace() const {
        return (size > 1) ? (size - 1 - available()) : 0;
    }
    
    bool isEmpty() const {
        return readPos == writePos;
    }
    
    // Write data to ring buffer (drops overflow)
    size_t write(const uint8_t* data, size_t len) {
        if (!buf || size < 2 || !data) return 0;
        
        size_t free = freeSpace();
        if (len > free) len = free;  // Drop overflow
        if (len == 0) return 0;
        
        size_t w = writePos;
        size_t firstPart = size - w;
        if (firstPart > len) firstPart = len;
        
        memcpy(buf + w, data, firstPart);
        if (len > firstPart) {
            memcpy(buf, data + firstPart, len - firstPart);
        }
        
        writePos = (w + len) % size;
        return len;
    }
    
    // Read data from ring buffer (zero-fills if not enough data)
    size_t read(uint8_t* out, size_t len, bool zeroFill = true) {
        if (!buf || size < 2 || !out) {
            if (zeroFill && out) memset(out, 0, len);
            return 0;
        }
        
        size_t avail = available();
        size_t toRead = (len > avail) ? avail : len;
        
        if (toRead > 0) {
            size_t r = readPos;
            size_t firstPart = size - r;
            if (firstPart > toRead) firstPart = toRead;
            
            memcpy(out, buf + r, firstPart);
            if (toRead > firstPart) {
                memcpy(out + firstPart, buf, toRead - firstPart);
            }
            
            readPos = (r + toRead) % size;
        }
        
        // Zero-fill remainder if requested
        if (zeroFill && toRead < len) {
            memset(out + toRead, 0, len - toRead);
        }
        
        return toRead;
    }
};

// -----------------------------------------------------------
// Audio Mixer Class
// -----------------------------------------------------------
class AudioMixer {
public:
    static AudioMixer& getInstance() {
        static AudioMixer instance;
        return instance;
    }
    
    bool init(I2SOutput* i2s) {
        if (m_initialized) return true;
        
        m_i2s = i2s;
        
        // Initialize ring buffers in PSRAM
        m_btRing.init(MIXER_RB_BT_BYTES, true);
        m_ovlRing.init(MIXER_RB_OVL_BYTES, true);
        
        if (!m_btRing.buf || !m_ovlRing.buf) {
            ESP_LOGE(TAG, "Failed to allocate mixer ring buffers");
            return false;
        }
        
        ESP_LOGI(TAG, "Mixer ring buffers: BT=%uKB, Overlay=%uKB",
                 (unsigned)(MIXER_RB_BT_BYTES / 1024),
                 (unsigned)(MIXER_RB_OVL_BYTES / 1024));
        
        // Allocate mix output buffer (internal RAM for DMA)
        m_mixBuf = (int32_t*)heap_caps_malloc(MIXER_CHUNK_FRAMES * 2 * sizeof(int32_t), 
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!m_mixBuf) {
            m_mixBuf = (int32_t*)heap_caps_malloc(MIXER_CHUNK_FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
        }
        if (!m_mixBuf) {
            ESP_LOGE(TAG, "Failed to allocate mix buffer");
            return false;
        }
        
        // Temp buffers for reading from ring buffers
        m_btTempBuf = (int32_t*)heap_caps_malloc(MIXER_CHUNK_FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
        m_ovlTempBuf = (int32_t*)heap_caps_malloc(MIXER_CHUNK_FRAMES * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
        
        if (!m_btTempBuf || !m_ovlTempBuf) {
            ESP_LOGE(TAG, "Failed to allocate temp buffers");
            return false;
        }
        
        // Start mixer task
        BaseType_t ret = xTaskCreatePinnedToCore(
            mixerTaskWrapper,
            "mixer",
            4096,
            this,
            configMAX_PRIORITIES - 2,  // High priority
            &m_mixerTask,
            0  // Run on core 0
        );
        
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create mixer task");
            return false;
        }
        
        m_initialized = true;
        ESP_LOGI(TAG, "Audio mixer initialized");
        return true;
    }
    
    // Push BT PCM data (32-bit stereo, from A2DP callback)
    void pushBtPcm(const uint8_t* data, size_t bytes) {
        if (!m_initialized || !data || bytes == 0) return;
        if (bytes % MIXER_BYTES_PER_FRAME != 0) return;  // Must be complete frames
        
        m_btRing.write(data, bytes);
        m_btActive = true;
    }
    
    // Push overlay PCM data (32-bit stereo, from WAV resampler)
    void pushOverlayPcm(const uint8_t* data, size_t bytes) {
        if (!m_initialized || !data || bytes == 0) return;
        if (bytes % MIXER_BYTES_PER_FRAME != 0) return;
        
        m_ovlRing.write(data, bytes);
    }
    
    // Start overlay (duck BT audio)
    void overlayBegin() {
        ESP_LOGI(TAG, "Overlay begin - ducking BT audio");
        m_overlayPlaying = true;
        m_overlayEndRequested = false;
        m_btTargetGainQ15 = m_duckGainQ15;
    }
    
    // Signal overlay finished (restore BT when ring drains)
    void overlayEndWhenDrained() {
        ESP_LOGI(TAG, "Overlay end requested - will restore when drained");
        m_overlayEndRequested = true;
    }
    
    // Check if overlay is currently playing
    bool isOverlayPlaying() const {
        return m_overlayPlaying;
    }
    
    // Set duck gain (0-32768 Q15)
    void setDuckGain(int16_t gainQ15) {
        if (gainQ15 < 0) gainQ15 = 0;
        if (gainQ15 > Q15_UNITY) gainQ15 = Q15_UNITY;
        m_duckGainQ15 = gainQ15;
    }
    
    // Check if mixer is bypassed (BT goes direct to I2S via pipeline)
    bool isBypassed() const {
        return m_bypassed;
    }
    
    // Enable/disable bypass mode (for compatibility with existing pipeline)
    void setBypass(bool bypass) {
        m_bypassed = bypass;
    }
    
    // Get BT ring buffer available bytes
    size_t getBtBufferLevel() const {
        return m_btRing.available();
    }
    
private:
    static constexpr const char* TAG = "AudioMixer";
    
    AudioMixer() = default;
    
    static void mixerTaskWrapper(void* param) {
        ((AudioMixer*)param)->mixerTask();
    }
    
    // Saturating add for 32-bit audio
    static inline int32_t sat32(int64_t x) {
        if (x > INT32_MAX) return INT32_MAX;
        if (x < INT32_MIN) return INT32_MIN;
        return (int32_t)x;
    }
    
    void mixerTask() {
        ESP_LOGI(TAG, "Mixer task started");
        
        while (true) {
            // If bypassed, sleep and let pipeline handle I2S directly
            if (m_bypassed) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            
            // Gain ramp (~30 steps to full transition)
            if (m_btCurrentGainQ15 != m_btTargetGainQ15) {
                int32_t diff = m_btTargetGainQ15 - m_btCurrentGainQ15;
                int32_t step = 1024;  // ~32 steps to ramp full scale
                
                if (diff > step) {
                    m_btCurrentGainQ15 += step;
                } else if (diff < -step) {
                    m_btCurrentGainQ15 -= step;
                } else {
                    m_btCurrentGainQ15 = m_btTargetGainQ15;
                }
            }
            
            // Read from both ring buffers
            size_t btBytes = MIXER_CHUNK_FRAMES * MIXER_BYTES_PER_FRAME;
            size_t ovlBytes = MIXER_CHUNK_FRAMES * MIXER_BYTES_PER_FRAME;
            
            size_t gotBt = m_btRing.read((uint8_t*)m_btTempBuf, btBytes, true);
            size_t gotOvl = m_ovlRing.read((uint8_t*)m_ovlTempBuf, ovlBytes, true);
            
            // Mix: out = BT * gain + overlay
            int16_t gain = m_btCurrentGainQ15;
            
            for (int i = 0; i < MIXER_CHUNK_FRAMES * 2; i++) {
                // Apply gain to BT
                int64_t btSample = ((int64_t)m_btTempBuf[i] * gain) >> 15;
                
                // Add overlay
                int64_t mixed = btSample + m_ovlTempBuf[i];
                
                m_mixBuf[i] = sat32(mixed);
            }
            
            // Restore BT gain when overlay finished AND ring drained
            if (m_overlayEndRequested && m_ovlRing.isEmpty()) {
                m_overlayEndRequested = false;
                m_overlayPlaying = false;
                m_btTargetGainQ15 = Q15_UNITY;
                ESP_LOGI(TAG, "Overlay complete - restoring BT audio");
            }
            
            // Write to I2S
            if (m_i2s && m_i2s->isInitialized()) {
                size_t written = m_i2s->write(m_mixBuf, MIXER_CHUNK_FRAMES * MIXER_BYTES_PER_FRAME);
                if (written == 0) {
                    // I2S busy or reconfiguring, wait a bit
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
    
    bool m_initialized = false;
    bool m_bypassed = true;  // Start in bypass mode for compatibility
    bool m_btActive = false;
    
    I2SOutput* m_i2s = nullptr;
    TaskHandle_t m_mixerTask = nullptr;
    
    MixerRingBuf m_btRing;
    MixerRingBuf m_ovlRing;
    
    int32_t* m_mixBuf = nullptr;
    int32_t* m_btTempBuf = nullptr;
    int32_t* m_ovlTempBuf = nullptr;
    
    volatile bool m_overlayPlaying = false;
    volatile bool m_overlayEndRequested = false;
    
    volatile int16_t m_btCurrentGainQ15 = Q15_UNITY;
    volatile int16_t m_btTargetGainQ15 = Q15_UNITY;
    volatile int16_t m_duckGainQ15 = Q15_DUCK_DEFAULT;
};

// -----------------------------------------------------------
// WAV Overlay Player (streams WAV with resampling)
// -----------------------------------------------------------
class WavOverlayPlayer {
public:
    static WavOverlayPlayer& getInstance() {
        static WavOverlayPlayer instance;
        return instance;
    }
    
    void init(AudioMixer* mixer, I2SOutput* i2s) {
        m_mixer = mixer;
        m_i2s = i2s;
    }
    
    // Play WAV file as overlay with ducking
    bool play(const char* path) {
        if (!m_mixer || !m_i2s) {
            ESP_LOGE(TAG, "Mixer not initialized");
            return false;
        }
        
        if (m_playing) {
            ESP_LOGW(TAG, "Already playing, stopping current");
            stop();
            vTaskDelay(pdMS_TO_TICKS(50));  // Let task clean up
        }
        
        strncpy(m_currentPath, path, sizeof(m_currentPath) - 1);
        m_currentPath[sizeof(m_currentPath) - 1] = '\0';
        m_stopRequested = false;
        m_playing = true;
        
        BaseType_t ret = xTaskCreatePinnedToCore(
            playTaskWrapper,
            "wav_ovl",
            4096,
            this,
            configMAX_PRIORITIES - 3,
            &m_playTask,
            1  // Core 1
        );
        
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create overlay task");
            m_playing = false;
            return false;
        }
        
        return true;
    }
    
    void stop() {
        m_stopRequested = true;
    }
    
    bool isPlaying() const {
        return m_playing;
    }
    
private:
    static constexpr const char* TAG = "WavOverlay";
    
    WavOverlayPlayer() = default;
    
    static void playTaskWrapper(void* param) {
        ((WavOverlayPlayer*)param)->playTask();
        vTaskDelete(NULL);
    }
    
    // WAV header structure
    struct WavHeader {
        char riff[4];
        uint32_t fileSize;
        char wave[4];
        char fmt[4];
        uint32_t fmtSize;
        uint16_t audioFormat;
        uint16_t numChannels;
        uint32_t sampleRate;
        uint32_t byteRate;
        uint16_t blockAlign;
        uint16_t bitsPerSample;
    };
    
    // Linear interpolation resampler state
    struct Resampler {
        float ratio;
        float pos;
        int16_t lastL, lastR;
        bool hasLast;
        bool stereo;
        
        void init(uint32_t inRate, uint32_t outRate, bool isStereo) {
            ratio = (float)inRate / (float)outRate;
            pos = 0.0f;
            lastL = lastR = 0;
            hasLast = false;
            stereo = isStereo;
        }
    };
    
    void playTask() {
        ESP_LOGI(TAG, "Playing overlay: %s", m_currentPath);
        
        FILE* f = fopen(m_currentPath, "rb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open: %s", m_currentPath);
            m_playing = false;
            return;
        }
        
        // Read WAV header
        WavHeader header;
        if (fread(&header, 1, sizeof(header), f) != sizeof(header)) {
            ESP_LOGE(TAG, "Failed to read WAV header");
            fclose(f);
            m_playing = false;
            return;
        }
        
        // Validate
        if (memcmp(header.riff, "RIFF", 4) != 0 ||
            memcmp(header.wave, "WAVE", 4) != 0 ||
            header.audioFormat != 1) {
            ESP_LOGE(TAG, "Invalid WAV file");
            fclose(f);
            m_playing = false;
            return;
        }
        
        uint32_t outRate = m_i2s->getSampleRate();
        ESP_LOGI(TAG, "WAV: %uHz %ubit %uch -> I2S %uHz",
                 (unsigned)header.sampleRate,
                 (unsigned)header.bitsPerSample,
                 (unsigned)header.numChannels,
                 (unsigned)outRate);
        
        // Find data chunk
        char chunkId[4];
        uint32_t chunkSize = 0;
        while (fread(chunkId, 1, 4, f) == 4) {
            fread(&chunkSize, 4, 1, f);
            if (memcmp(chunkId, "data", 4) == 0) break;
            fseek(f, chunkSize, SEEK_CUR);
        }
        
        // Initialize resampler
        Resampler rs;
        bool isStereo = (header.numChannels == 2);
        rs.init(header.sampleRate, outRate, isStereo);
        
        // Allocate buffers (prefer PSRAM)
        const size_t inChunkFrames = (header.sampleRate * 20) / 1000;  // 20ms
        const size_t maxOutFrames = (inChunkFrames * outRate / header.sampleRate) + 16;
        
        uint8_t* inBuf = (uint8_t*)heap_caps_malloc(inChunkFrames * header.blockAlign, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!inBuf) inBuf = (uint8_t*)heap_caps_malloc(inChunkFrames * header.blockAlign, MALLOC_CAP_8BIT);
        
        int16_t* inS16 = (int16_t*)heap_caps_malloc(inChunkFrames * header.numChannels * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!inS16) inS16 = (int16_t*)heap_caps_malloc(inChunkFrames * header.numChannels * 2, MALLOC_CAP_8BIT);
        
        int32_t* outS32 = (int32_t*)heap_caps_malloc(maxOutFrames * 2 * sizeof(int32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outS32) outS32 = (int32_t*)heap_caps_malloc(maxOutFrames * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
        
        if (!inBuf || !inS16 || !outS32) {
            ESP_LOGE(TAG, "Buffer alloc failed");
            if (inBuf) heap_caps_free(inBuf);
            if (inS16) heap_caps_free(inS16);
            if (outS32) heap_caps_free(outS32);
            fclose(f);
            m_mixer->overlayEndWhenDrained();
            m_playing = false;
            return;
        }
        
        // Start overlay (duck BT)
        m_mixer->overlayBegin();
        
        // Stream and resample
        size_t bytesRead = 0;
        while (bytesRead < chunkSize && !m_stopRequested) {
            size_t toRead = inChunkFrames * header.blockAlign;
            size_t remaining = chunkSize - bytesRead;
            if (toRead > remaining) toRead = remaining;
            
            size_t actualRead = fread(inBuf, 1, toRead, f);
            if (actualRead == 0) break;
            
            bytesRead += actualRead;
            size_t inFrames = actualRead / header.blockAlign;
            
            // Convert to 16-bit
            if (header.bitsPerSample == 16) {
                memcpy(inS16, inBuf, inFrames * header.numChannels * 2);
            } else if (header.bitsPerSample == 8) {
                for (size_t i = 0; i < inFrames * header.numChannels; i++) {
                    inS16[i] = ((int16_t)inBuf[i] - 128) << 8;
                }
            }
            
            // Resample to output rate
            size_t outFrames = resample(rs, inS16, inFrames, outS32, maxOutFrames, header.numChannels);
            
            // Push to mixer overlay ring
            if (outFrames > 0) {
                m_mixer->pushOverlayPcm((uint8_t*)outS32, outFrames * MIXER_BYTES_PER_FRAME);
            }
            
            taskYIELD();
        }
        
        // Cleanup
        heap_caps_free(inBuf);
        heap_caps_free(inS16);
        heap_caps_free(outS32);
        fclose(f);
        
        // Signal overlay complete
        m_mixer->overlayEndWhenDrained();
        m_playing = false;
        
        ESP_LOGI(TAG, "Overlay playback complete");
    }
    
    // Linear interpolation resampler
    size_t resample(Resampler& rs, const int16_t* in, size_t inFrames,
                    int32_t* out, size_t outCapacity, uint16_t channels) {
        size_t outCount = 0;
        
        if (!rs.hasLast && inFrames > 0) {
            rs.lastL = in[0];
            rs.lastR = (channels == 2) ? in[1] : in[0];
            rs.hasLast = true;
        }
        
        while (outCount < outCapacity) {
            uint32_t idx = (uint32_t)rs.pos;
            uint32_t next = idx + 1;
            
            if (next >= inFrames) break;
            
            float frac = rs.pos - (float)idx;
            
            int16_t l0, r0, l1, r1;
            
            if (channels == 2) {
                if (idx == 0 && rs.hasLast) {
                    l0 = rs.lastL;
                    r0 = rs.lastR;
                } else {
                    l0 = in[idx * 2];
                    r0 = in[idx * 2 + 1];
                }
                l1 = in[next * 2];
                r1 = in[next * 2 + 1];
            } else {
                if (idx == 0 && rs.hasLast) {
                    l0 = r0 = rs.lastL;
                } else {
                    l0 = r0 = in[idx];
                }
                l1 = r1 = in[next];
            }
            
            // Linear interpolation
            float sampleL = (1.0f - frac) * l0 + frac * l1;
            float sampleR = (1.0f - frac) * r0 + frac * r1;
            
            // Expand 16-bit -> 32-bit (left align)
            out[outCount * 2] = (int32_t)(sampleL * 65536.0f);
            out[outCount * 2 + 1] = (int32_t)(sampleR * 65536.0f);
            
            outCount++;
            rs.pos += rs.ratio;
        }
        
        // Save state for next chunk
        uint32_t consumed = (uint32_t)rs.pos;
        rs.pos -= (float)consumed;
        
        if (inFrames > 0) {
            if (channels == 2) {
                rs.lastL = in[(inFrames - 1) * 2];
                rs.lastR = in[(inFrames - 1) * 2 + 1];
            } else {
                rs.lastL = rs.lastR = in[inFrames - 1];
            }
        }
        
        return outCount;
    }
    
    AudioMixer* m_mixer = nullptr;
    I2SOutput* m_i2s = nullptr;
    TaskHandle_t m_playTask = nullptr;
    
    char m_currentPath[128] = {0};
    volatile bool m_playing = false;
    volatile bool m_stopRequested = false;
};
