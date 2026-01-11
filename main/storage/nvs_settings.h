#pragma once

// -----------------------------------------------------------
// NVS Settings - persistent storage for device configuration
// -----------------------------------------------------------

#include <string>
#include <stdint.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "../config/app_config.h"

class NVSSettings {
public:
    // Settings structure
    struct Settings {
        std::string deviceName;
        uint8_t controlByte;      // bit0=bassBoost, bit1=channelFlip, bit2=bypass
        int8_t eqBassDB;
        int8_t eqMidDB;
        int8_t eqTrebleDB;

        Settings() 
            : deviceName(APP_DEFAULT_DEVICE_NAME)
            , controlByte(0)
            , eqBassDB(0)
            , eqMidDB(0)
            , eqTrebleDB(0) 
        {}
    };

    NVSSettings() {}

    // Load all settings from NVS (stores internally)
    bool load() {
        return load(m_settings);
    }

    // Get current settings
    void getControl(bool &bassBoost, bool &channelFlip, bool &bypass) const {
        bassBoost = (m_settings.controlByte & 0x01) != 0;
        channelFlip = (m_settings.controlByte & 0x02) != 0;
        bypass = (m_settings.controlByte & 0x04) != 0;
    }

    void getEQ(int8_t &bass, int8_t &mid, int8_t &treble) const {
        bass = m_settings.eqBassDB;
        mid = m_settings.eqMidDB;
        treble = m_settings.eqTrebleDB;
    }

    void getDeviceName(std::string &name) const {
        name = m_settings.deviceName;
    }

    // Save control flags
    bool saveControl(bool bassBoost, bool channelFlip, bool bypass) {
        uint8_t ctrl = 0;
        if (bassBoost) ctrl |= 0x01;
        if (channelFlip) ctrl |= 0x02;
        if (bypass) ctrl |= 0x04;
        m_settings.controlByte = ctrl;
        return saveControl(ctrl);
    }

    // Load all settings from NVS (explicit struct)
    bool load(Settings &settings) {
        nvs_handle_t h;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NVS open failed, using defaults");
            settings = Settings();
            return false;
        }

        // Device name
        char buf[32];
        size_t len = sizeof(buf);
        err = nvs_get_str(h, NVS_KEY_DEVNAME, buf, &len);
        if (err == ESP_OK && len > 0) {
            settings.deviceName.assign(buf, len - 1);
        } else {
            settings.deviceName = APP_DEFAULT_DEVICE_NAME;
        }

        // Control byte
        uint8_t ctrl = 0;
        if (nvs_get_u8(h, NVS_KEY_CTRL, &ctrl) == ESP_OK) {
            settings.controlByte = ctrl;
        }

        // EQ
        int8_t b = 0, m = 0, t = 0;
        if (nvs_get_i8(h, NVS_KEY_EQ_BASS, &b) == ESP_OK) settings.eqBassDB = b;
        if (nvs_get_i8(h, NVS_KEY_EQ_MID, &m) == ESP_OK) settings.eqMidDB = m;
        if (nvs_get_i8(h, NVS_KEY_EQ_TREB, &t) == ESP_OK) settings.eqTrebleDB = t;

        nvs_close(h);

        ESP_LOGI(TAG, "NVS loaded: name='%s', ctrl=0x%02x, EQ(b,m,t)=(%d,%d,%d)",
                 settings.deviceName.c_str(), settings.controlByte,
                 settings.eqBassDB, settings.eqMidDB, settings.eqTrebleDB);
        return true;
    }

    // Save control byte
    bool saveControl(uint8_t ctrl) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_u8(h, NVS_KEY_CTRL, ctrl);
        nvs_commit(h);
        nvs_close(h);
        return true;
    }

    // Save EQ settings
    bool saveEQ(int8_t bass, int8_t mid, int8_t treble) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_i8(h, NVS_KEY_EQ_BASS, bass);
        nvs_set_i8(h, NVS_KEY_EQ_MID, mid);
        nvs_set_i8(h, NVS_KEY_EQ_TREB, treble);
        nvs_commit(h);
        nvs_close(h);
        return true;
    }

    // Save device name
    bool saveDeviceName(const std::string &name) {
        m_settings.deviceName = name;
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_str(h, NVS_KEY_DEVNAME, name.c_str());
        nvs_commit(h);
        nvs_close(h);
        return true;
    }
    
    bool saveDeviceName(const char* name) {
        return saveDeviceName(std::string(name));
    }

private:
    static constexpr const char* TAG = "NVS";
    Settings m_settings;
};
