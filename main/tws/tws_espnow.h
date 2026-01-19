#pragma once

// -----------------------------------------------------------
// TWS (True Wireless Stereo) using ESP-NOW
// Primary: Receives A2DP from phone, forwards audio to secondary via ESP-NOW
// Secondary: Receives audio from primary via ESP-NOW, outputs to I2S
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

// -----------------------------------------------------------
// TWS Configuration
// -----------------------------------------------------------

// TWS roles
enum class TwsRole : uint8_t {
    Disabled = 0,   // TWS mode off - normal single speaker operation
    Primary = 1,    // Receives A2DP, forwards to secondary
    Secondary = 2   // Receives audio from primary via ESP-NOW
};

// TWS channel assignment
enum class TwsChannel : uint8_t {
    Stereo = 0,     // Both channels (for testing)
    Left = 1,       // Left channel only
    Right = 2       // Right channel only
};

// ESP-NOW packet types
enum class TwsPacketType : uint8_t {
    Audio = 0x01,       // Audio data packet
    Sync = 0x02,        // Sync/timing packet
    Pair = 0x03,        // Pairing request
    PairAck = 0x04,     // Pairing acknowledgment
    Config = 0x05,      // Configuration (sample rate, etc.)
    Heartbeat = 0x06    // Keep-alive
};

// Audio packet header (sent before audio data)
struct __attribute__((packed)) TwsAudioHeader {
    uint8_t type;           // TwsPacketType::Audio
    uint8_t seqNum;         // Sequence number for ordering
    uint16_t frames;        // Number of audio frames
    uint32_t timestamp;     // Timestamp for sync (microseconds, lower 32 bits)
    uint8_t bits;           // Bits per sample (16 or 32)
    uint8_t channels;       // 1 = mono (after split), 2 = stereo
    uint16_t reserved;      // Padding for alignment
};

// Sync packet (for timing alignment)
struct __attribute__((packed)) TwsSyncPacket {
    uint8_t type;           // TwsPacketType::Sync
    uint8_t reserved[3];
    uint32_t timestamp;     // Current timestamp
    uint32_t playbackDelay; // Delay before playback (microseconds)
};

// Config packet (sent after pairing)
struct __attribute__((packed)) TwsConfigPacket {
    uint8_t type;           // TwsPacketType::Config
    uint8_t channel;        // TwsChannel for this device
    uint16_t reserved;
    uint32_t sampleRate;    // Audio sample rate
    uint32_t syncDelayUs;   // Sync delay in microseconds
};

// Pairing packet
struct __attribute__((packed)) TwsPairPacket {
    uint8_t type;           // TwsPacketType::Pair or PairAck
    uint8_t role;           // Sender's role
    uint8_t mac[6];         // Sender's MAC address
    uint32_t version;       // Protocol version for compatibility
};

// -----------------------------------------------------------
// TWS Manager Class
// -----------------------------------------------------------

class TwsManager {
public:
    static constexpr const char* TAG = "TWS";
    
    // ESP-NOW limits: 250 bytes max payload
    // Header = 12 bytes, so max audio = 238 bytes
    // For 16-bit stereo: 238 / 4 = 59 frames per packet
    // For 16-bit mono (after split): 238 / 2 = 119 frames per packet
    static constexpr size_t ESPNOW_MAX_PAYLOAD = 250;
    static constexpr size_t AUDIO_HEADER_SIZE = sizeof(TwsAudioHeader);
    static constexpr size_t MAX_AUDIO_BYTES = ESPNOW_MAX_PAYLOAD - AUDIO_HEADER_SIZE;
    
    // Sync buffer size (milliseconds of audio to buffer for sync)
    static constexpr uint32_t SYNC_BUFFER_MS = 50;
    
    // Heartbeat interval
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
    
    // Protocol version
    static constexpr uint32_t PROTOCOL_VERSION = 1;

    TwsManager()
        : m_role(TwsRole::Disabled)
        , m_channel(TwsChannel::Stereo)
        , m_paired(false)
        , m_connected(false)
        , m_seqNum(0)
        , m_sampleRate(44100)
        , m_syncDelayUs(SYNC_BUFFER_MS * 1000)
        , m_lastHeartbeat(0)
        , m_rxQueue(nullptr)
        , m_initialized(false)
    {
        memset(m_peerMac, 0xFF, 6);  // Broadcast initially
    }

    ~TwsManager() {
        deinit();
    }

    // Initialize TWS with specified role
    bool init(TwsRole role, TwsChannel channel = TwsChannel::Stereo) {
        if (role == TwsRole::Disabled) {
            ESP_LOGI(TAG, "TWS disabled");
            return true;
        }
        
        m_role = role;
        m_channel = channel;
        
        ESP_LOGI(TAG, "Initializing TWS as %s, channel=%s",
                 role == TwsRole::Primary ? "PRIMARY" : "SECONDARY",
                 channel == TwsChannel::Left ? "LEFT" : 
                 channel == TwsChannel::Right ? "RIGHT" : "STEREO");
        
        // Initialize WiFi in station mode (required for ESP-NOW)
        if (!initWifi()) {
            ESP_LOGE(TAG, "WiFi init failed");
            return false;
        }
        
        // Initialize ESP-NOW
        if (esp_now_init() != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW init failed");
            return false;
        }
        
        // Register callbacks
        esp_now_register_recv_cb(onReceiveStatic);
        esp_now_register_send_cb(onSendStatic);
        
        // For secondary: create receive queue
        if (role == TwsRole::Secondary) {
            m_rxQueue = xQueueCreate(32, ESPNOW_MAX_PAYLOAD);
            if (!m_rxQueue) {
                ESP_LOGE(TAG, "Failed to create RX queue");
                return false;
            }
        }
        
        // Add broadcast peer for pairing
        esp_now_peer_info_t peerInfo = {};
        memset(peerInfo.peer_addr, 0xFF, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
        
        m_initialized = true;
        ESP_LOGI(TAG, "TWS initialized successfully");
        
        return true;
    }

    void deinit() {
        if (m_initialized) {
            esp_now_deinit();
            if (m_rxQueue) {
                vQueueDelete(m_rxQueue);
                m_rxQueue = nullptr;
            }
            m_initialized = false;
        }
        m_role = TwsRole::Disabled;
    }

    // Start pairing mode (broadcast pairing request)
    void startPairing() {
        if (!m_initialized || m_role == TwsRole::Disabled) return;
        
        ESP_LOGI(TAG, "Starting TWS pairing...");
        
        TwsPairPacket pkt = {};
        pkt.type = static_cast<uint8_t>(TwsPacketType::Pair);
        pkt.role = static_cast<uint8_t>(m_role);
        pkt.version = PROTOCOL_VERSION;
        
        // Get our MAC address
        esp_wifi_get_mac(WIFI_IF_STA, pkt.mac);
        
        // Broadcast pairing request
        uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_send(broadcast, (uint8_t*)&pkt, sizeof(pkt));
    }

    // Set peer MAC address (skip pairing, use known address)
    void setPeerMac(const uint8_t* mac) {
        memcpy(m_peerMac, mac, 6);
        
        // Add as ESP-NOW peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
        // Remove if already exists
        esp_now_del_peer(mac);
        esp_now_add_peer(&peerInfo);
        
        m_paired = true;
        m_connected = true;
        
        ESP_LOGI(TAG, "Peer set: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Send audio data (called by primary after decoding)
    // Input: decoded PCM audio (stereo or mono)
    // This will split channels if needed and send via ESP-NOW
    void sendAudio(const int16_t* samples, uint32_t frames, uint8_t bits, uint8_t channels) {
        if (!m_initialized || m_role != TwsRole::Primary || !m_paired) return;
        
        // For TWS, we send the appropriate channel to secondary
        // Primary plays local channel, secondary plays remote channel
        
        uint8_t txBuf[ESPNOW_MAX_PAYLOAD];
        TwsAudioHeader* hdr = (TwsAudioHeader*)txBuf;
        uint8_t* audioData = txBuf + AUDIO_HEADER_SIZE;
        
        // Calculate how many frames we can send per packet
        size_t bytesPerFrame = (bits <= 16) ? 2 : 4;  // Mono after split
        size_t maxFramesPerPacket = MAX_AUDIO_BYTES / bytesPerFrame;
        
        uint32_t timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        
        const int16_t* src = samples;
        uint32_t remaining = frames;
        
        while (remaining > 0) {
            uint32_t sendFrames = (remaining > maxFramesPerPacket) ? maxFramesPerPacket : remaining;
            
            // Fill header
            hdr->type = static_cast<uint8_t>(TwsPacketType::Audio);
            hdr->seqNum = m_seqNum++;
            hdr->frames = (uint16_t)sendFrames;
            hdr->timestamp = timestamp;
            hdr->bits = 16;  // Always 16-bit for ESP-NOW efficiency
            hdr->channels = 1;  // Mono after split
            hdr->reserved = 0;
            
            // Extract the channel to send to secondary
            // If secondary is RIGHT, send right channel; if LEFT, send left channel
            // Primary keeps the opposite channel
            int16_t* dst = (int16_t*)audioData;
            
            if (channels == 1) {
                // Mono input - send as-is
                memcpy(dst, src, sendFrames * sizeof(int16_t));
                src += sendFrames;
            } else {
                // Stereo input - extract one channel for secondary
                // Default: Secondary gets RIGHT channel, Primary keeps LEFT
                int channelOffset = (m_channel == TwsChannel::Left) ? 0 : 1;
                for (uint32_t i = 0; i < sendFrames; i++) {
                    dst[i] = src[i * 2 + channelOffset];
                }
                src += sendFrames * 2;
            }
            
            // Send packet
            size_t pktLen = AUDIO_HEADER_SIZE + sendFrames * sizeof(int16_t);
            esp_now_send(m_peerMac, txBuf, pktLen);
            
            remaining -= sendFrames;
            timestamp += (sendFrames * 1000000) / m_sampleRate;  // Advance timestamp
        }
    }

    // Send sync packet to align playback timing
    void sendSync(uint32_t playbackDelay) {
        if (!m_initialized || m_role != TwsRole::Primary || !m_paired) return;
        
        TwsSyncPacket pkt = {};
        pkt.type = static_cast<uint8_t>(TwsPacketType::Sync);
        pkt.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        pkt.playbackDelay = playbackDelay;
        
        esp_now_send(m_peerMac, (uint8_t*)&pkt, sizeof(pkt));
    }

    // Send configuration to secondary
    void sendConfig() {
        if (!m_initialized || m_role != TwsRole::Primary || !m_paired) return;
        
        TwsConfigPacket pkt = {};
        pkt.type = static_cast<uint8_t>(TwsPacketType::Config);
        pkt.channel = static_cast<uint8_t>(m_channel == TwsChannel::Left ? TwsChannel::Right : TwsChannel::Left);
        pkt.sampleRate = m_sampleRate;
        pkt.syncDelayUs = m_syncDelayUs;
        
        esp_now_send(m_peerMac, (uint8_t*)&pkt, sizeof(pkt));
        ESP_LOGI(TAG, "Config sent: rate=%u, delay=%u us", m_sampleRate, m_syncDelayUs);
    }

    // Receive audio data (called by secondary's audio task)
    // Returns number of frames received, 0 if no data
    uint32_t receiveAudio(int16_t* outSamples, uint32_t maxFrames, uint8_t* outBits, uint32_t* outTimestamp) {
        if (!m_initialized || m_role != TwsRole::Secondary || !m_rxQueue) return 0;
        
        uint8_t rxBuf[ESPNOW_MAX_PAYLOAD];
        
        if (xQueueReceive(m_rxQueue, rxBuf, pdMS_TO_TICKS(20)) != pdTRUE) {
            return 0;
        }
        
        TwsAudioHeader* hdr = (TwsAudioHeader*)rxBuf;
        
        if (hdr->type != static_cast<uint8_t>(TwsPacketType::Audio)) {
            // Handle other packet types
            handleNonAudioPacket(rxBuf);
            return 0;
        }
        
        uint32_t frames = hdr->frames;
        if (frames > maxFrames) frames = maxFrames;
        
        // Copy audio data
        memcpy(outSamples, rxBuf + AUDIO_HEADER_SIZE, frames * sizeof(int16_t));
        
        if (outBits) *outBits = hdr->bits;
        if (outTimestamp) *outTimestamp = hdr->timestamp;
        
        return frames;
    }

    // Check if TWS is active
    bool isEnabled() const { return m_role != TwsRole::Disabled && m_initialized; }
    bool isPrimary() const { return m_role == TwsRole::Primary; }
    bool isSecondary() const { return m_role == TwsRole::Secondary; }
    bool isPaired() const { return m_paired; }
    bool isConnected() const { return m_connected; }
    
    TwsRole getRole() const { return m_role; }
    TwsChannel getChannel() const { return m_channel; }
    
    void setSampleRate(uint32_t rate) { m_sampleRate = rate; }
    uint32_t getSampleRate() const { return m_sampleRate; }
    
    void setSyncDelay(uint32_t delayUs) { m_syncDelayUs = delayUs; }
    uint32_t getSyncDelay() const { return m_syncDelayUs; }

    // Get local channel (what this device should play)
    TwsChannel getLocalChannel() const {
        if (m_role == TwsRole::Primary) {
            // Primary plays the opposite of what it sends to secondary
            return (m_channel == TwsChannel::Right) ? TwsChannel::Left : TwsChannel::Right;
        } else {
            // Secondary plays what primary sends it
            return m_channel;
        }
    }

    // Singleton access
    static TwsManager& getInstance() {
        static TwsManager instance;
        return instance;
    }

private:
    bool initWifi() {
        // WiFi must be initialized for ESP-NOW
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        
        esp_err_t ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = esp_wifi_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        // Set WiFi channel (should match on both devices)
        ret = esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WiFi set channel failed: %s", esp_err_to_name(ret));
        }
        
        ESP_LOGI(TAG, "WiFi initialized for ESP-NOW");
        return true;
    }

    void handleNonAudioPacket(const uint8_t* data) {
        TwsPacketType type = static_cast<TwsPacketType>(data[0]);
        
        switch (type) {
            case TwsPacketType::Sync: {
                const TwsSyncPacket* pkt = (const TwsSyncPacket*)data;
                m_syncDelayUs = pkt->playbackDelay;
                ESP_LOGD(TAG, "Sync received: delay=%u us", m_syncDelayUs);
                break;
            }
            case TwsPacketType::Config: {
                const TwsConfigPacket* pkt = (const TwsConfigPacket*)data;
                m_channel = static_cast<TwsChannel>(pkt->channel);
                m_sampleRate = pkt->sampleRate;
                m_syncDelayUs = pkt->syncDelayUs;
                ESP_LOGI(TAG, "Config received: channel=%d, rate=%u, delay=%u",
                         pkt->channel, pkt->sampleRate, pkt->syncDelayUs);
                break;
            }
            case TwsPacketType::Heartbeat:
                m_lastHeartbeat = esp_timer_get_time();
                m_connected = true;
                break;
            default:
                break;
        }
    }

    // Static callback wrappers for ESP-NOW
    static void onReceiveStatic(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
        getInstance().onReceive(info->src_addr, data, len);
    }
    
    static void onSendStatic(const uint8_t* mac, esp_now_send_status_t status) {
        getInstance().onSend(mac, status);
    }

    void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
        if (len < 1) return;
        
        TwsPacketType type = static_cast<TwsPacketType>(data[0]);
        
        // Handle pairing
        if (type == TwsPacketType::Pair) {
            handlePairRequest(mac, data, len);
            return;
        }
        
        if (type == TwsPacketType::PairAck) {
            handlePairAck(mac, data, len);
            return;
        }
        
        // For secondary: queue audio and other packets
        if (m_role == TwsRole::Secondary && m_rxQueue) {
            // Verify packet is from our paired peer
            if (!m_paired || memcmp(mac, m_peerMac, 6) != 0) {
                return;  // Ignore packets from unknown sources
            }
            
            if (type == TwsPacketType::Audio) {
                // Queue audio packet (drop if full - prefer fresh data)
                xQueueSend(m_rxQueue, data, 0);
            } else {
                handleNonAudioPacket(data);
            }
        }
    }

    void onSend(const uint8_t* mac, esp_now_send_status_t status) {
        if (status != ESP_NOW_SEND_SUCCESS) {
            // Track send failures for connection monitoring
            // Could implement retry logic here
        }
    }

    void handlePairRequest(const uint8_t* mac, const uint8_t* data, int len) {
        if (len < sizeof(TwsPairPacket)) return;
        
        const TwsPairPacket* pkt = (const TwsPairPacket*)data;
        
        // Only respond if roles are compatible (Primary-Secondary)
        TwsRole senderRole = static_cast<TwsRole>(pkt->role);
        if ((m_role == TwsRole::Primary && senderRole == TwsRole::Secondary) ||
            (m_role == TwsRole::Secondary && senderRole == TwsRole::Primary)) {
            
            ESP_LOGI(TAG, "Pair request from %s: %02X:%02X:%02X:%02X:%02X:%02X",
                     senderRole == TwsRole::Primary ? "PRIMARY" : "SECONDARY",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            
            // Store peer MAC and add as ESP-NOW peer
            setPeerMac(mac);
            
            // Send acknowledgment
            TwsPairPacket ack = {};
            ack.type = static_cast<uint8_t>(TwsPacketType::PairAck);
            ack.role = static_cast<uint8_t>(m_role);
            ack.version = PROTOCOL_VERSION;
            esp_wifi_get_mac(WIFI_IF_STA, ack.mac);
            
            esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
            
            ESP_LOGI(TAG, "TWS paired!");
        }
    }

    void handlePairAck(const uint8_t* mac, const uint8_t* data, int len) {
        if (len < sizeof(TwsPairPacket)) return;
        
        if (!m_paired) {
            ESP_LOGI(TAG, "Pair acknowledged by: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            
            setPeerMac(mac);
            
            // If we're primary, send config
            if (m_role == TwsRole::Primary) {
                sendConfig();
            }
        }
    }

    TwsRole m_role;
    TwsChannel m_channel;
    bool m_paired;
    bool m_connected;
    uint8_t m_seqNum;
    uint32_t m_sampleRate;
    uint32_t m_syncDelayUs;
    int64_t m_lastHeartbeat;
    uint8_t m_peerMac[6];
    QueueHandle_t m_rxQueue;
    bool m_initialized;
};
