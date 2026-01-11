#pragma once

// -----------------------------------------------------------
// BLE GATT Service - manages Bluetooth Low Energy services
// Handles advertising, connections, and characteristic I/O
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "../config/app_config.h"

// Forward declarations for callbacks
class BleGattService;
static BleGattService* s_bleInstance = nullptr;

class BleGattService {
public:
    // Control state callback - receives control byte when written
    using ControlCallback = void(*)(uint8_t controlByte);
    // EQ callback - receives bass, mid, treble dB values
    using EqCallback = void(*)(int8_t bass, int8_t mid, int8_t treble);
    // Name callback - receives new device name
    using NameCallback = void(*)(const char* name, size_t len);
    // OTA control callback - receives OTA command packets
    using OtaCtrlCallback = void(*)(const uint8_t* data, size_t len);
    // OTA data callback - receives OTA firmware data
    using OtaDataCallback = void(*)(const uint8_t* data, size_t len);

    BleGattService()
        : m_gattsIf(0)
        , m_connId(0)
        , m_connected(false)
        , m_advConfigDone(0)
        // Service handles
        , m_levelsServiceHandle(0)
        , m_controlServiceHandle(0)
        // Characteristic handles
        , m_levelsCharHandle(0)
        , m_controlCharHandle(0)
        , m_eqCharHandle(0)
        , m_nameCharHandle(0)
        , m_fwCharHandle(0)
        , m_otaCtrlCharHandle(0)
        , m_otaDataCharHandle(0)
        // Callbacks
        , m_controlCb(nullptr)
        , m_eqCb(nullptr)
        , m_nameCb(nullptr)
        , m_otaCtrlCb(nullptr)
        , m_otaDataCb(nullptr)
    {
        memset(m_uuidLevelsService, 0, 16);
        memset(m_uuidLevelsChar, 0, 16);
        memset(m_uuidControlService, 0, 16);
        memset(m_uuidControlChar, 0, 16);
        memset(m_uuidEqChar, 0, 16);
        memset(m_uuidNameChar, 0, 16);
        memset(m_uuidFwChar, 0, 16);
        memset(m_uuidOtaCtrlChar, 0, 16);
        memset(m_uuidOtaDataChar, 0, 16);
        
        // Initialize characteristic values
        memset(m_controlValue, 0, sizeof(m_controlValue));
        memset(m_eqValue, 0, sizeof(m_eqValue));
        memset(m_nameValue, 0, sizeof(m_nameValue));
        memset(m_fwValue, 0, sizeof(m_fwValue));
        memset(m_levelsValue, 0, sizeof(m_levelsValue));
    }

    void setCallbacks(ControlCallback ctrlCb, EqCallback eqCb, NameCallback nameCb,
                      OtaCtrlCallback otaCtrlCb, OtaDataCallback otaDataCb) {
        m_controlCb = ctrlCb;
        m_eqCb = eqCb;
        m_nameCb = nameCb;
        m_otaCtrlCb = otaCtrlCb;
        m_otaDataCb = otaDataCb;
    }

    bool init(const char* deviceName, const char* fwVersion,
              uint8_t controlByte, int8_t bassDb, int8_t midDb, int8_t trebleDb) {
        
        s_bleInstance = this;
        
        // Store device name
        strncpy(m_nameValue, deviceName, sizeof(m_nameValue) - 1);
        strncpy(m_fwValue, fwVersion, sizeof(m_fwValue) - 1);
        m_controlValue[0] = controlByte;
        m_eqValue[0] = (uint8_t)bassDb;
        m_eqValue[1] = (uint8_t)midDb;
        m_eqValue[2] = (uint8_t)trebleDb;

        // Parse UUIDs
        uuid128FromString(APP_BLE_SERVICE_UUID_LEVELS, m_uuidLevelsService);
        uuid128FromString(APP_BLE_CHAR_UUID_LEVELS, m_uuidLevelsChar);
        uuid128FromString(APP_BLE_SERVICE_UUID_CONTROL, m_uuidControlService);
        uuid128FromString(APP_BLE_CHAR_UUID_CONTROL, m_uuidControlChar);
        uuid128FromString(APP_BLE_CHAR_UUID_EQ, m_uuidEqChar);
        uuid128FromString(APP_BLE_CHAR_UUID_DEVNAME, m_uuidNameChar);
        uuid128FromString(APP_BLE_CHAR_UUID_FWVER, m_uuidFwChar);
        uuid128FromString(APP_BLE_CHAR_UUID_OTA_CTRL, m_uuidOtaCtrlChar);
        uuid128FromString(APP_BLE_CHAR_UUID_OTA_DATA, m_uuidOtaDataChar);

        // Init Bluetooth controller
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
        ESP_ERROR_CHECK(esp_bluedroid_init());
        ESP_ERROR_CHECK(esp_bluedroid_enable());

        // Register callbacks
        ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
        ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gattsEventHandler));
        ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

        ESP_LOGI(TAG, "BLE GATT initialized");
        return true;
    }

    bool isConnected() const { return m_connected; }

    // Update characteristic values and notify connected client
    void updateLevels(int l30, int l60, int l100) {
        int len = snprintf(m_levelsValue, sizeof(m_levelsValue), "%d,%d,%d", l30, l60, l100);
        if (m_connected && m_levelsCharHandle && m_gattsIf) {
            esp_err_t ret = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_levelsCharHandle,
                                        len, (uint8_t*)m_levelsValue, false);
            (void)ret;  // Ignore errors for high-frequency level updates
        }
    }

    void updateControl(uint8_t controlByte) {
        m_controlValue[0] = controlByte;
        if (m_connected && m_controlCharHandle && m_gattsIf) {
            esp_err_t ret = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_controlCharHandle,
                                        1, m_controlValue, false);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Control notify failed: %d", ret);
            }
        }
    }

    void updateEq(int8_t bass, int8_t mid, int8_t treble) {
        m_eqValue[0] = (uint8_t)bass;
        m_eqValue[1] = (uint8_t)mid;
        m_eqValue[2] = (uint8_t)treble;
        if (m_connected && m_eqCharHandle && m_gattsIf) {
            esp_err_t ret = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_eqCharHandle,
                                        3, m_eqValue, false);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "EQ notify failed: %d", ret);
            }
        }
    }

    void updateName(const char* name) {
        strncpy(m_nameValue, name, sizeof(m_nameValue) - 1);
        m_nameValue[sizeof(m_nameValue) - 1] = '\0';
        notifyName();
    }

    void updateFw(const char* fw) {
        strncpy(m_fwValue, fw, sizeof(m_fwValue) - 1);
        m_fwValue[sizeof(m_fwValue) - 1] = '\0';
        notifyFw();
    }

    void notifyFw() {
        if (m_connected && m_fwCharHandle && m_gattsIf) {
            esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_fwCharHandle,
                                        strlen(m_fwValue), (uint8_t*)m_fwValue, false);
        }
    }

    void notifyName() {
        if (m_connected && m_nameCharHandle && m_gattsIf) {
            esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_nameCharHandle,
                                        strlen(m_nameValue), (uint8_t*)m_nameValue, false);
        }
    }

    void notifyOtaCtrl(const char* msg) {
        if (m_connected && m_otaCtrlCharHandle && m_gattsIf) {
            esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_otaCtrlCharHandle,
                                        strlen(msg), (uint8_t*)msg, false);
        }
    }

    // Get control byte value
    uint8_t getControlValue() const { return m_controlValue[0]; }

private:
    static constexpr const char* TAG = "BLE";
    static constexpr uint8_t ADV_CONFIG_FLAG = 0x01;
    static constexpr uint8_t SCAN_RSP_CONFIG_FLAG = 0x02;

    // UUID parsing helper
    static void uuid128FromString(const char* str, uint8_t* out) {
        // Parse UUID string like "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
        // Output is little-endian (reversed)
        uint8_t temp[16];
        memset(temp, 0, 16);
        int idx = 0;
        for (int i = 0; str[i] && idx < 32; i++) {
            char c = str[i];
            if (c == '-') continue;
            int val = 0;
            if (c >= '0' && c <= '9') val = c - '0';
            else if (c >= 'a' && c <= 'f') val = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
            else continue;
            
            if ((idx & 1) == 0) {
                temp[idx >> 1] = (uint8_t)(val << 4);
            } else {
                temp[idx >> 1] |= (uint8_t)val;
            }
            idx++;
        }
        // Reverse for little-endian
        for (int i = 0; i < 16; i++) {
            out[i] = temp[15 - i];
        }
    }

    static bool uuidEqual128(const esp_bt_uuid_t& uuid, const uint8_t* raw) {
        if (uuid.len != ESP_UUID_LEN_128) return false;
        return memcmp(uuid.uuid.uuid128, raw, 16) == 0;
    }

    static bool uuidEqual128Raw(const uint8_t* a, const uint8_t* b) {
        return memcmp(a, b, 16) == 0;
    }

    // Static callback wrappers
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
        if (s_bleInstance) s_bleInstance->handleGapEvent(event, param);
    }

    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t* param) {
        if (s_bleInstance) s_bleInstance->handleGattsEvent(event, gatts_if, param);
    }

    void handleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
        switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            m_advConfigDone &= ~ADV_CONFIG_FLAG;
            if (m_advConfigDone == 0) {
                startAdvertising();
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            m_advConfigDone &= ~SCAN_RSP_CONFIG_FLAG;
            if (m_advConfigDone == 0) {
                startAdvertising();
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: 0x%x", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started");
            }
            break;
        default:
            break;
        }
    }

    void handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                          esp_ble_gatts_cb_param_t* param) {
        switch (event) {
        case ESP_GATTS_REG_EVT:
            handleRegEvent(gatts_if, param);
            break;
        case ESP_GATTS_CREATE_EVT:
            handleCreateEvent(gatts_if, param);
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            handleAddCharEvent(gatts_if, param);
            break;
        case ESP_GATTS_CONNECT_EVT:
            handleConnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            handleDisconnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_READ_EVT:
            handleReadEvent(gatts_if, param);
            break;
        case ESP_GATTS_WRITE_EVT:
            handleWriteEvent(gatts_if, param);
            break;
        default:
            break;
        }
    }

    void handleRegEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_gattsIf = gatts_if;
        
        // Configure advertising
        esp_ble_adv_data_t adv_data = {};
        adv_data.set_scan_rsp = false;
        adv_data.include_name = false;  // Name in scan response
        adv_data.include_txpower = true;
        adv_data.min_interval = 0x0006;
        adv_data.max_interval = 0x0010;
        adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
        adv_data.service_uuid_len = 16;
        adv_data.p_service_uuid = m_uuidControlService;

        esp_ble_adv_data_t scan_rsp = {};
        scan_rsp.set_scan_rsp = true;
        scan_rsp.include_name = true;
        scan_rsp.include_txpower = true;

        esp_ble_gap_set_device_name(m_nameValue);
        m_advConfigDone = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp);

        // Create services
        createLevelsService(gatts_if);
        createControlService(gatts_if);
    }

    void createLevelsService(esp_gatt_if_t gatts_if) {
        esp_gatt_srvc_id_t svc = {};
        svc.is_primary = true;
        svc.id.inst_id = 0x00;
        svc.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc.id.uuid.uuid.uuid128, m_uuidLevelsService, 16);
        esp_ble_gatts_create_service(gatts_if, &svc, 4);
    }

    void createControlService(esp_gatt_if_t gatts_if) {
        esp_gatt_srvc_id_t svc = {};
        svc.is_primary = true;
        svc.id.inst_id = 0x01;
        svc.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc.id.uuid.uuid.uuid128, m_uuidControlService, 16);
        esp_ble_gatts_create_service(gatts_if, &svc, 20);
    }

    void handleCreateEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        bool isLevels = uuidEqual128Raw(param->create.service_id.id.uuid.uuid.uuid128, m_uuidLevelsService);
        bool isControl = uuidEqual128Raw(param->create.service_id.id.uuid.uuid.uuid128, m_uuidControlService);

        static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };

        if (isLevels) {
            m_levelsServiceHandle = param->create.service_handle;
            esp_ble_gatts_start_service(m_levelsServiceHandle);
            addCharacteristic(m_levelsServiceHandle, m_uuidLevelsChar, ESP_GATT_PERM_READ,
                             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
        } else if (isControl) {
            m_controlServiceHandle = param->create.service_handle;
            esp_ble_gatts_start_service(m_controlServiceHandle);
            
            // Add all control service characteristics
            addCharacteristic(m_controlServiceHandle, m_uuidControlChar,
                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            addCharacteristic(m_controlServiceHandle, m_uuidEqChar,
                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            addCharacteristic(m_controlServiceHandle, m_uuidNameChar,
                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            addCharacteristic(m_controlServiceHandle, m_uuidFwChar,
                             ESP_GATT_PERM_READ,
                             ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            addCharacteristic(m_controlServiceHandle, m_uuidOtaCtrlChar,
                             ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            addCharacteristic(m_controlServiceHandle, m_uuidOtaDataChar,
                             ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR);
        }
    }

    void addCharacteristic(uint16_t svcHandle, const uint8_t* uuid128, esp_gatt_perm_t perm, uint8_t prop) {
        static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
        esp_bt_uuid_t uuid = {};
        uuid.len = ESP_UUID_LEN_128;
        memcpy(uuid.uuid.uuid128, uuid128, 16);
        esp_attr_value_t val = {};
        val.attr_max_len = 512;
        val.attr_len = 0;
        esp_ble_gatts_add_char(svcHandle, &uuid, perm, (esp_gatt_char_prop_t)prop, &val, &rspByApp);
    }

    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        uint16_t handle = param->add_char.attr_handle;
        
        if (uuidEqual128(param->add_char.char_uuid, m_uuidLevelsChar)) {
            m_levelsCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidControlChar)) {
            m_controlCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidEqChar)) {
            m_eqCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidNameChar)) {
            m_nameCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidFwChar)) {
            m_fwCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidOtaCtrlChar)) {
            m_otaCtrlCharHandle = handle;
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidOtaDataChar)) {
            m_otaDataCharHandle = handle;
        }

        // Add CCCD for characteristics that support notify
        if (handle != m_otaDataCharHandle) {
            esp_bt_uuid_t cccdUuid = {};
            cccdUuid.len = ESP_UUID_LEN_16;
            cccdUuid.uuid.uuid16 = 0x2902;
            static esp_attr_control_t cccdRsp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
            esp_ble_gatts_add_char_descr(param->add_char.service_handle, &cccdUuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, &cccdRsp);
        }
    }

    void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connId = param->connect.conn_id;
        m_connected = true;
        ESP_LOGI(TAG, "Client connected, conn_id=%d", m_connId);

        // Create a task to send initial notifications - don't block the callback
        xTaskCreate([](void* arg) {
            BleGattService* self = (BleGattService*)arg;
            
            // Wait for client to set up subscriptions (CCCD writes)
            vTaskDelay(pdMS_TO_TICKS(500));
            
            if (!self->m_connected) {
                vTaskDelete(NULL);
                return;
            }
            
            // Send each characteristic with delay between them
            self->notifyFw();
            vTaskDelay(pdMS_TO_TICKS(100));
            
            if (!self->m_connected) { vTaskDelete(NULL); return; }
            self->notifyName();
            vTaskDelay(pdMS_TO_TICKS(100));
            
            if (!self->m_connected) { vTaskDelete(NULL); return; }
            self->updateEq((int8_t)self->m_eqValue[0], (int8_t)self->m_eqValue[1], (int8_t)self->m_eqValue[2]);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            if (!self->m_connected) { vTaskDelete(NULL); return; }
            // Send control with retries
            self->updateControl(self->m_controlValue[0]);
            vTaskDelay(pdMS_TO_TICKS(200));
            
            if (!self->m_connected) { vTaskDelete(NULL); return; }
            // Retry control notification
            self->updateControl(self->m_controlValue[0]);
            
            ESP_LOGI("BLE", "Initial notifications sent");
            vTaskDelete(NULL);
        }, "ble_notify", 2048, this, 5, NULL);
    }

    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connected = false;
        m_connId = 0;
        ESP_LOGI(TAG, "Client disconnected");
        startAdvertising();
    }

    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        esp_gatt_rsp_t rsp = {};
        rsp.attr_value.handle = param->read.handle;

        if (param->read.handle == m_controlCharHandle) {
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = m_controlValue[0];
        } else if (param->read.handle == m_eqCharHandle) {
            rsp.attr_value.len = 3;
            memcpy(rsp.attr_value.value, m_eqValue, 3);
        } else if (param->read.handle == m_nameCharHandle) {
            size_t len = strlen(m_nameValue);
            if (len > sizeof(rsp.attr_value.value)) len = sizeof(rsp.attr_value.value);
            memcpy(rsp.attr_value.value, m_nameValue, len);
            rsp.attr_value.len = len;
        } else if (param->read.handle == m_fwCharHandle) {
            size_t len = strlen(m_fwValue);
            if (len > sizeof(rsp.attr_value.value)) len = sizeof(rsp.attr_value.value);
            memcpy(rsp.attr_value.value, m_fwValue, len);
            rsp.attr_value.len = len;
        } else if (param->read.handle == m_levelsCharHandle) {
            size_t len = strlen(m_levelsValue);
            if (len > sizeof(rsp.attr_value.value)) len = sizeof(rsp.attr_value.value);
            memcpy(rsp.attr_value.value, m_levelsValue, len);
            rsp.attr_value.len = len;
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
    }

    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        uint16_t handle = param->write.handle;
        uint8_t* data = param->write.value;
        uint16_t len = param->write.len;

        if (handle == m_controlCharHandle && len >= 1) {
            m_controlValue[0] = data[0];
            if (m_controlCb) m_controlCb(data[0]);
        } else if (handle == m_eqCharHandle && len >= 3) {
            memcpy(m_eqValue, data, 3);
            if (m_eqCb) m_eqCb((int8_t)data[0], (int8_t)data[1], (int8_t)data[2]);
        } else if (handle == m_nameCharHandle && len >= 1) {
            size_t copyLen = (len < sizeof(m_nameValue) - 1) ? len : sizeof(m_nameValue) - 1;
            memcpy(m_nameValue, data, copyLen);
            m_nameValue[copyLen] = '\0';
            if (m_nameCb) m_nameCb(m_nameValue, copyLen);
        } else if (handle == m_otaCtrlCharHandle && len >= 1) {
            ESP_LOGI(TAG, "OTA CTRL write: len=%u, first=%02X", len, data[0]);
            if (m_otaCtrlCb) m_otaCtrlCb(data, len);
        } else if (handle == m_otaDataCharHandle && len >= 1) {
            if (m_otaDataCb) m_otaDataCb(data, len);
        }

        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
    }

    void startAdvertising() {
        esp_ble_adv_params_t adv_params = {};
        adv_params.adv_int_min = 0x20;
        adv_params.adv_int_max = 0x40;
        adv_params.adv_type = ADV_TYPE_IND;
        adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        adv_params.channel_map = ADV_CHNL_ALL;
        adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
        esp_ble_gap_start_advertising(&adv_params);
    }

    // GATT interface
    esp_gatt_if_t m_gattsIf;
    uint16_t m_connId;
    bool m_connected;
    uint8_t m_advConfigDone;

    // Service handles
    uint16_t m_levelsServiceHandle;
    uint16_t m_controlServiceHandle;

    // Characteristic handles
    uint16_t m_levelsCharHandle;
    uint16_t m_controlCharHandle;
    uint16_t m_eqCharHandle;
    uint16_t m_nameCharHandle;
    uint16_t m_fwCharHandle;
    uint16_t m_otaCtrlCharHandle;
    uint16_t m_otaDataCharHandle;

    // UUIDs (little-endian)
    uint8_t m_uuidLevelsService[16];
    uint8_t m_uuidLevelsChar[16];
    uint8_t m_uuidControlService[16];
    uint8_t m_uuidControlChar[16];
    uint8_t m_uuidEqChar[16];
    uint8_t m_uuidNameChar[16];
    uint8_t m_uuidFwChar[16];
    uint8_t m_uuidOtaCtrlChar[16];
    uint8_t m_uuidOtaDataChar[16];

    // Characteristic values
    uint8_t m_controlValue[1];
    uint8_t m_eqValue[3];
    char m_nameValue[64];
    char m_fwValue[32];
    char m_levelsValue[32];

    // Callbacks
    ControlCallback m_controlCb;
    EqCallback m_eqCb;
    NameCallback m_nameCb;
    OtaCtrlCallback m_otaCtrlCb;
    OtaDataCallback m_otaDataCb;
};
