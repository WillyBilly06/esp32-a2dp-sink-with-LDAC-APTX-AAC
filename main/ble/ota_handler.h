#pragma once

// -----------------------------------------------------------
// OTA Handler - manages over-the-air firmware updates via BLE
// Implements pre-begin buffering and packet coalescing
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "../config/app_config.h"
#include "../idf_update.h"  // IdfUpdate wrapper

// OTA command types
enum class OtaCommand : uint8_t {
    BEGIN = 0x01,
    DATA = 0x02,
    END = 0x03,
    ABORT = 0x04
};

// OTA state machine
enum class OtaState : uint8_t {
    IDLE = 0,
    BUFFERING,    // Pre-begin data accumulation
    UPDATING,     // Active OTA write
    FINISHING,    // Waiting for completion
    ERROR
};

class OtaHandler {
public:
    // Progress callback: (received_bytes, total_bytes)
    using ProgressCallback = void(*)(uint32_t received, uint32_t total);

    OtaHandler() 
        : m_state(OtaState::IDLE)
        , m_buffer(nullptr)
        , m_bufferPos(0)
        , m_totalSize(0)
        , m_written(0)
        , m_packetsInFlight(0)
        , m_progressCb(nullptr)
        , m_mutex(nullptr)
        , m_lastProgressPct(0)
    {
    }

    ~OtaHandler() {
        cleanup();
        if (m_mutex) vSemaphoreDelete(m_mutex);
    }

    bool init() {
        m_mutex = xSemaphoreCreateMutex();
        if (!m_mutex) {
            ESP_LOGE(TAG, "Failed to create OTA mutex");
            return false;
        }
        
        // Pre-allocate buffer in SPIRAM
        m_buffer = (uint8_t*)heap_caps_malloc(OTA_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!m_buffer) {
            ESP_LOGW(TAG, "SPIRAM failed, using internal");
            m_buffer = (uint8_t*)heap_caps_malloc(OTA_BUFFER_SIZE, MALLOC_CAP_8BIT);
        }
        if (!m_buffer) {
            ESP_LOGE(TAG, "Failed to allocate OTA buffer");
            return false;
        }
        
        ESP_LOGI(TAG, "OTA handler initialized");
        return true;
    }

    void setProgressCallback(ProgressCallback cb) {
        m_progressCb = cb;
    }

    // Handle incoming OTA packet
    int handlePacket(const uint8_t *data, size_t len) {
        if (len < 1) return -1;

        OtaCommand cmd = static_cast<OtaCommand>(data[0]);
        const uint8_t *payload = data + 1;
        size_t payloadLen = len - 1;

        lock();
        int result = 0;

        switch (cmd) {
            case OtaCommand::BEGIN:
                result = handleBegin(payload, payloadLen);
                break;
            case OtaCommand::DATA:
                result = handleData(payload, payloadLen);
                break;
            case OtaCommand::END:
                result = handleEnd();
                break;
            case OtaCommand::ABORT:
                result = handleAbort();
                break;
            default:
                ESP_LOGW(TAG, "Unknown OTA command: 0x%02X", data[0]);
                result = -1;
                break;
        }

        unlock();
        return result;
    }

    // Signal packet acknowledgement (for flow control)
    void onPacketAck() {
        lock();
        if (m_packetsInFlight > 0) {
            m_packetsInFlight--;
        }
        unlock();
    }

    // Get current state
    OtaState getState() const { return m_state; }
    uint32_t getTotalSize() const { return m_totalSize; }
    uint32_t getWritten() const { return m_written; }
    uint8_t getProgressPercent() const {
        if (m_totalSize == 0) return 0;
        return (uint8_t)((uint64_t)m_written * 100 / m_totalSize);
    }

    // Wait for in-flight packets to drain
    bool waitForPackets(uint32_t timeoutMs = 1000) {
        uint32_t start = millis32();
        while (m_packetsInFlight > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            if ((millis32() - start) > timeoutMs) {
                ESP_LOGW(TAG, "Timeout waiting for %u packets", (unsigned)m_packetsInFlight);
                return false;
            }
        }
        return true;
    }

private:
    static constexpr const char* TAG = "OTA";
    static constexpr size_t OTA_BUFFER_SIZE = APP_OTA_PRE_BEGIN_BUFFER;

    void lock() {
        if (m_mutex) xSemaphoreTake(m_mutex, portMAX_DELAY);
    }

    void unlock() {
        if (m_mutex) xSemaphoreGive(m_mutex);
    }

    static uint32_t millis32() {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    int handleBegin(const uint8_t *payload, size_t len) {
        if (len < 4) {
            ESP_LOGE(TAG, "BEGIN packet too short");
            return -1;
        }

        // Read total size (little endian)
        m_totalSize = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
        
        ESP_LOGI(TAG, "OTA BEGIN: total size = %u bytes", (unsigned)m_totalSize);

        // Validate size
        const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
        if (!part) {
            ESP_LOGE(TAG, "No OTA partition available");
            m_state = OtaState::ERROR;
            return -1;
        }
        if (m_totalSize > part->size) {
            ESP_LOGE(TAG, "Image too large: %u > %u", (unsigned)m_totalSize, (unsigned)part->size);
            m_state = OtaState::ERROR;
            return -1;
        }

        // Reset state
        m_bufferPos = 0;
        m_written = 0;
        m_packetsInFlight = 0;
        m_lastProgressPct = 0;
        m_state = OtaState::BUFFERING;

        return 0;
    }

    int handleData(const uint8_t *payload, size_t len) {
        if (m_state == OtaState::IDLE || m_state == OtaState::ERROR) {
            ESP_LOGW(TAG, "DATA received in invalid state");
            return -1;
        }

        m_packetsInFlight++;

        // Buffer data for pre-begin phase or coalescing
        if (m_bufferPos + len <= OTA_BUFFER_SIZE) {
            memcpy(m_buffer + m_bufferPos, payload, len);
            m_bufferPos += len;
        }

        // Start actual OTA if we have enough data
        if (m_state == OtaState::BUFFERING && m_bufferPos >= OTA_BUFFER_SIZE / 2) {
            if (!startOtaWrite()) {
                return -1;
            }
        }

        // Flush buffer if near full
        if (m_state == OtaState::UPDATING && m_bufferPos >= OTA_BUFFER_SIZE * 3 / 4) {
            flushBuffer();
        }

        return 0;
    }

    int handleEnd() {
        ESP_LOGI(TAG, "OTA END received, %u in buffer", (unsigned)m_bufferPos);

        // Wait for in-flight packets
        unlock();
        waitForPackets(2000);
        lock();

        // Start OTA if still buffering
        if (m_state == OtaState::BUFFERING) {
            if (!startOtaWrite()) {
                return -1;
            }
        }

        // Flush remaining data
        if (m_bufferPos > 0) {
            flushBuffer();
        }

        // Complete OTA
        if (IdfUpdate.end(true)) {
            ESP_LOGI(TAG, "OTA complete! Written %u bytes", (unsigned)m_written);
            m_state = OtaState::IDLE;
            return 1;  // Signal reboot needed
        } else {
            ESP_LOGE(TAG, "OTA finalization failed");
            m_state = OtaState::ERROR;
            return -1;
        }
    }

    int handleAbort() {
        ESP_LOGW(TAG, "OTA ABORT received");
        cleanup();
        m_state = OtaState::IDLE;
        return 0;
    }

    bool startOtaWrite() {
        ESP_LOGI(TAG, "Starting OTA write with %u buffered bytes", (unsigned)m_bufferPos);

        if (!IdfUpdate.begin(m_totalSize)) {
            ESP_LOGE(TAG, "IdfUpdate.begin failed");
            m_state = OtaState::ERROR;
            return false;
        }

        m_state = OtaState::UPDATING;
        return true;
    }

    void flushBuffer() {
        if (m_bufferPos == 0) return;

        size_t wrote = IdfUpdate.write(m_buffer, m_bufferPos);
        if (wrote != m_bufferPos) {
            ESP_LOGW(TAG, "Short write: %u/%u", (unsigned)wrote, (unsigned)m_bufferPos);
        }

        m_written += wrote;
        m_bufferPos = 0;

        // Report progress
        uint8_t pct = getProgressPercent();
        if (pct != m_lastProgressPct) {
            m_lastProgressPct = pct;
            ESP_LOGI(TAG, "Progress: %u%% (%u/%u)", pct, (unsigned)m_written, (unsigned)m_totalSize);
            if (m_progressCb) {
                m_progressCb(m_written, m_totalSize);
            }
        }
    }

    void cleanup() {
        m_bufferPos = 0;
        m_written = 0;
        m_totalSize = 0;
        m_packetsInFlight = 0;
        m_lastProgressPct = 0;
        // Don't free buffer - keep it allocated for next OTA
    }

    OtaState m_state;
    uint8_t *m_buffer;
    size_t m_bufferPos;
    uint32_t m_totalSize;
    uint32_t m_written;
    volatile uint32_t m_packetsInFlight;
    ProgressCallback m_progressCb;
    SemaphoreHandle_t m_mutex;
    uint8_t m_lastProgressPct;
};
