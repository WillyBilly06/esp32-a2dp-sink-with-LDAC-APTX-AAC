/**
 * @file recovery_main.cpp
 * @brief BLE OTA Recovery Firmware using Bluedroid Stack
 * 
 * Uses Bluedroid for maximum compatibility with Android app.
 * Classic BT + A2DP are compiled in (required by modified ESP-IDF)
 * but only BLE GATT is used for OTA operations.
 */

#include <cstring>
#include <cstdlib>
#include <cinttypes>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
}

static const char* TAG = "RECOVERY";

// ============================================================
// BLE UUIDs (MUST match Android app and main firmware)
// ============================================================
#define BLE_SERVICE_UUID_CONTROL  "12345678-1234-1234-1234-1234567890ad"
#define BLE_CHAR_UUID_OTA_CTRL    "12345678-1234-1234-1234-1234567890b1"
#define BLE_CHAR_UUID_OTA_DATA    "12345678-1234-1234-1234-1234567890b2"
#define BLE_CHAR_UUID_FWVER       "12345678-1234-1234-1234-1234567890b3"

// ============================================================
// Configuration
// ============================================================
static const char* DEVICE_NAME = "ESP32-Recovery";
static const char* RECOVERY_VERSION = "RECOVERY v1.0";

// ============================================================
// OTA State
// ============================================================
static struct {
    bool           active;
    uint32_t       totalSize;
    uint32_t       received;
    int64_t        checkPassedTime;
    esp_ota_handle_t handle;
    const esp_partition_t* partition;
} g_ota = {};

// ============================================================
// BLE State
// ============================================================
static esp_gatt_if_t g_gattsIf = 0;
static uint16_t g_connId = 0;
static bool g_connected = false;
static uint8_t g_advConfigDone = 0;

static uint16_t g_serviceHandle = 0;
static uint16_t g_otaCtrlCharHandle = 0;
static uint16_t g_otaCtrlCccdHandle = 0;
static uint16_t g_otaDataCharHandle = 0;
static uint16_t g_fwCharHandle = 0;
static uint16_t g_fwCccdHandle = 0;

static uint8_t g_uuidService[16];
static uint8_t g_uuidOtaCtrl[16];
static uint8_t g_uuidOtaData[16];
static uint8_t g_uuidFw[16];

static char g_fwValue[32] = "RECOVERY v1.0";

// Track which characteristic we're adding CCCD for
static uint16_t g_lastCharHandle = 0;

// Notification enable flags
static bool g_otaCtrlNotifyEnabled = false;
static bool g_fwNotifyEnabled = false;

static constexpr uint8_t ADV_CONFIG_FLAG = 0x01;
static constexpr uint8_t SCAN_RSP_CONFIG_FLAG = 0x02;

// ============================================================
// UUID Parsing
// ============================================================
static void uuid128FromString(const char* str, uint8_t* out) {
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
    for (int i = 0; i < 16; i++) {
        out[i] = temp[15 - i];
    }
}

static bool uuidEqual128(const esp_bt_uuid_t& uuid, const uint8_t* raw) {
    if (uuid.len != ESP_UUID_LEN_128) return false;
    return memcmp(uuid.uuid.uuid128, raw, 16) == 0;
}

// ============================================================
// OTA Notification Helper
// ============================================================
static void notifyOtaCtrl(const char* msg) {
    if (!g_connected || !g_otaCtrlCharHandle || !g_gattsIf) {
        ESP_LOGW(TAG, "Cannot notify - not connected");
        return;
    }
    if (!g_otaCtrlNotifyEnabled) {
        ESP_LOGW(TAG, "OTA Ctrl notifications not enabled by client");
        // Send anyway - some clients don't properly enable CCCD
    }
    
    // Small delay to ensure client is ready to receive
    vTaskDelay(pdMS_TO_TICKS(50));
    
    esp_err_t err = esp_ble_gatts_send_indicate(g_gattsIf, g_connId, g_otaCtrlCharHandle,
                                                strlen(msg), (uint8_t*)msg, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Notify failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "OTA notify: %s", msg);
    }
}

// ============================================================
// OTA Command Handler
// ============================================================
static void handleOtaCommand(const uint8_t* data, size_t len) {
    if (len < 1) return;
    
    ESP_LOGI(TAG, "OTA CTRL: len=%u, data='%.*s'", (unsigned)len, (int)len, (char*)data);
    
    // BEGIN:<size>
    if (data[0] == 'B' && len >= 6 && memcmp(data, "BEGIN:", 6) == 0) {
        char sizeBuf[16] = {0};
        size_t copyLen = (len - 6 < sizeof(sizeBuf) - 1) ? (len - 6) : (sizeof(sizeBuf) - 1);
        memcpy(sizeBuf, data + 6, copyLen);
        uint32_t size = (uint32_t)atoi(sizeBuf);
        
        ESP_LOGI(TAG, "OTA BEGIN: %u bytes", (unsigned)size);
        
        // Abort any previous OTA in progress
        if (g_ota.active) {
            ESP_LOGW(TAG, "Aborting previous OTA");
            esp_ota_abort(g_ota.handle);
            g_ota.active = false;
            g_ota.received = 0;
        }
        
        g_ota.partition = esp_ota_get_next_update_partition(NULL);
        if (!g_ota.partition) {
            ESP_LOGE(TAG, "No OTA partition");
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        ESP_LOGI(TAG, "Target: %s @ 0x%08" PRIx32, g_ota.partition->label, (uint32_t)g_ota.partition->address);
        
        if (size > g_ota.partition->size) {
            ESP_LOGE(TAG, "Too large: %u > %u", (unsigned)size, (unsigned)g_ota.partition->size);
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        esp_err_t err = esp_ota_begin(g_ota.partition, size, &g_ota.handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        g_ota.active = true;
        g_ota.totalSize = size;
        g_ota.received = 0;
        g_ota.checkPassedTime = 0;
        
        notifyOtaCtrl("BEGIN_OK");
        return;
    }
    
    // CHECK:<size>
    if (data[0] == 'C' && len >= 6 && memcmp(data, "CHECK:", 6) == 0) {
        uint32_t expectedSize = 0;
        for (size_t i = 6; i < len && data[i] >= '0' && data[i] <= '9'; i++) {
            expectedSize = expectedSize * 10 + (data[i] - '0');
        }
        
        ESP_LOGI(TAG, "OTA CHECK: %u / %u", (unsigned)g_ota.received, (unsigned)expectedSize);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        if (g_ota.received >= expectedSize) {
            notifyOtaCtrl("CHECK_OK");
            g_ota.checkPassedTime = esp_timer_get_time();
            
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(3000));
                if (g_ota.active && g_ota.checkPassedTime > 0) {
                    ESP_LOGW(TAG, "Auto-finalizing...");
                    esp_err_t err = esp_ota_end(g_ota.handle);
                    if (err == ESP_OK) {
                        err = esp_ota_set_boot_partition(g_ota.partition);
                        if (err == ESP_OK) {
                            notifyOtaCtrl("END_OK");
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            esp_restart();
                        }
                    }
                    notifyOtaCtrl("END_ERR");
                    g_ota.active = false;
                }
                vTaskDelete(NULL);
            }, "ota_end", 4096, NULL, 5, NULL);
        } else {
            char resp[32];
            snprintf(resp, sizeof(resp), "CHECK_FAIL:%u", (unsigned)g_ota.received);
            notifyOtaCtrl(resp);
        }
        return;
    }
    
    // END
    if (data[0] == 'E' && len >= 3 && memcmp(data, "END", 3) == 0) {
        ESP_LOGI(TAG, "OTA END");
        g_ota.checkPassedTime = 0;
        
        esp_err_t err = esp_ota_end(g_ota.handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(g_ota.partition);
            if (err == ESP_OK) {
                notifyOtaCtrl("END_OK");
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }
        notifyOtaCtrl("END_ERR");
        g_ota.active = false;
        return;
    }
    
    // ABORT
    if (data[0] == 'A' && len >= 5 && memcmp(data, "ABORT", 5) == 0) {
        ESP_LOGW(TAG, "OTA ABORT");
        if (g_ota.active) esp_ota_abort(g_ota.handle);
        g_ota.active = false;
        g_ota.received = 0;
        notifyOtaCtrl("ABORT_OK");
        return;
    }
}

// ============================================================
// OTA Data Handler
// ============================================================
static void handleOtaData(const uint8_t* data, size_t len) {
    if (!g_ota.active || len == 0 || len == 1) return;
    
    esp_err_t err = esp_ota_write(g_ota.handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(err));
        return;
    }
    
    g_ota.received += len;
    
    uint8_t pct = (g_ota.totalSize > 0) ? (uint8_t)((uint64_t)g_ota.received * 100 / g_ota.totalSize) : 0;
    static uint8_t lastPct = 255;
    if (pct != lastPct && (pct % 5 == 0 || pct == 100)) {
        ESP_LOGI(TAG, "OTA: %u%% (%u/%u)", pct, (unsigned)g_ota.received, (unsigned)g_ota.totalSize);
        lastPct = pct;
    }
}

// ============================================================
// BLE Event Handlers
// ============================================================
static void startAdvertising() {
    esp_ble_adv_params_t adv_params = {};
    adv_params.adv_int_min = 0x20;
    adv_params.adv_int_max = 0x40;
    adv_params.adv_type = ADV_TYPE_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    esp_ble_gap_start_advertising(&adv_params);
}

static void addCharacteristic(uint16_t svcHandle, const uint8_t* uuid128, esp_gatt_perm_t perm, uint8_t prop) {
    esp_bt_uuid_t uuid = {};
    uuid.len = ESP_UUID_LEN_128;
    memcpy(uuid.uuid.uuid128, uuid128, 16);
    esp_attr_value_t val = {};
    val.attr_max_len = 512;
    val.attr_len = 0;
    static esp_attr_control_t ctrl = { .auto_rsp = ESP_GATT_RSP_BY_APP };
    esp_ble_gatts_add_char(svcHandle, &uuid, perm, (esp_gatt_char_prop_t)prop, &val, &ctrl);
}

static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        g_advConfigDone &= ~ADV_CONFIG_FLAG;
        if (g_advConfigDone == 0) startAdvertising();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        g_advConfigDone &= ~SCAN_RSP_CONFIG_FLAG;
        if (g_advConfigDone == 0) startAdvertising();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising started - connect to '%s'", DEVICE_NAME);
        } else {
            ESP_LOGE(TAG, "Advertising failed: 0x%x", param->adv_start_cmpl.status);
        }
        break;
    default:
        break;
    }
}

static void handleRegEvent(esp_gatt_if_t gatts_if) {
    g_gattsIf = gatts_if;
    
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = false;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x0006;
    adv_data.max_interval = 0x0010;
    adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
    adv_data.service_uuid_len = 16;
    adv_data.p_service_uuid = g_uuidService;

    esp_ble_adv_data_t scan_rsp = {};
    scan_rsp.set_scan_rsp = true;
    scan_rsp.include_name = true;
    scan_rsp.include_txpower = true;

    esp_ble_gap_set_device_name(DEVICE_NAME);
    g_advConfigDone = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_config_adv_data(&scan_rsp);

    esp_gatt_srvc_id_t svc = {};
    svc.is_primary = true;
    svc.id.inst_id = 0x00;
    svc.id.uuid.len = ESP_UUID_LEN_128;
    memcpy(svc.id.uuid.uuid.uuid128, g_uuidService, 16);
    esp_ble_gatts_create_service(gatts_if, &svc, 12);
}

static void handleCreateEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    g_serviceHandle = param->create.service_handle;
    esp_ble_gatts_start_service(g_serviceHandle);
    ESP_LOGI(TAG, "Service created, handle=%d", g_serviceHandle);
    
    addCharacteristic(g_serviceHandle, g_uuidOtaCtrl, ESP_GATT_PERM_WRITE,
                     ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
    addCharacteristic(g_serviceHandle, g_uuidOtaData, ESP_GATT_PERM_WRITE,
                     ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR);
    addCharacteristic(g_serviceHandle, g_uuidFw, ESP_GATT_PERM_READ,
                     ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
}

static void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    uint16_t handle = param->add_char.attr_handle;
    
    if (uuidEqual128(param->add_char.char_uuid, g_uuidOtaCtrl)) {
        g_otaCtrlCharHandle = handle;
        g_lastCharHandle = handle;
        ESP_LOGI(TAG, "OTA Ctrl handle: %d", handle);
    } else if (uuidEqual128(param->add_char.char_uuid, g_uuidOtaData)) {
        g_otaDataCharHandle = handle;
        g_lastCharHandle = handle;
        ESP_LOGI(TAG, "OTA Data handle: %d", handle);
    } else if (uuidEqual128(param->add_char.char_uuid, g_uuidFw)) {
        g_fwCharHandle = handle;
        g_lastCharHandle = handle;
        ESP_LOGI(TAG, "FW Version handle: %d", handle);
    }
    
    // Add CCCD for notify chars (not for OTA Data which is write-only)
    if (handle != g_otaDataCharHandle) {
        esp_bt_uuid_t cccd = {};
        cccd.len = ESP_UUID_LEN_16;
        cccd.uuid.uuid16 = 0x2902;
        static esp_attr_control_t ctrl = { .auto_rsp = ESP_GATT_RSP_BY_APP };
        esp_ble_gatts_add_char_descr(param->add_char.service_handle, &cccd,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, &ctrl);
    }
}

static void handleAddDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    // Track CCCD handles
    if (param->add_char_descr.descr_uuid.len == ESP_UUID_LEN_16 &&
        param->add_char_descr.descr_uuid.uuid.uuid16 == 0x2902) {
        uint16_t cccdHandle = param->add_char_descr.attr_handle;
        
        if (g_lastCharHandle == g_otaCtrlCharHandle) {
            g_otaCtrlCccdHandle = cccdHandle;
            ESP_LOGI(TAG, "OTA Ctrl CCCD handle: %d", cccdHandle);
        } else if (g_lastCharHandle == g_fwCharHandle) {
            g_fwCccdHandle = cccdHandle;
            ESP_LOGI(TAG, "FW CCCD handle: %d", cccdHandle);
        }
    }
}

static void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    esp_gatt_rsp_t rsp = {};
    rsp.attr_value.handle = param->read.handle;

    if (param->read.handle == g_fwCharHandle) {
        size_t len = strlen(g_fwValue);
        memcpy(rsp.attr_value.value, g_fwValue, len);
        rsp.attr_value.len = len;
        ESP_LOGI(TAG, "FW read: %s", g_fwValue);
    } else if (param->read.handle == g_otaCtrlCccdHandle || param->read.handle == g_fwCccdHandle) {
        // Return current CCCD value
        bool enabled = (param->read.handle == g_otaCtrlCccdHandle) ? g_otaCtrlNotifyEnabled : g_fwNotifyEnabled;
        rsp.attr_value.value[0] = enabled ? 0x01 : 0x00;
        rsp.attr_value.value[1] = 0x00;
        rsp.attr_value.len = 2;
        ESP_LOGI(TAG, "CCCD read: %s", enabled ? "enabled" : "disabled");
    }

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

static void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    // Handle CCCD writes (notification enable/disable)
    if (param->write.handle == g_otaCtrlCccdHandle) {
        g_otaCtrlNotifyEnabled = (param->write.len >= 1 && param->write.value[0] != 0);
        ESP_LOGI(TAG, "OTA Ctrl notifications %s", g_otaCtrlNotifyEnabled ? "ENABLED" : "disabled");
    } else if (param->write.handle == g_fwCccdHandle) {
        g_fwNotifyEnabled = (param->write.len >= 1 && param->write.value[0] != 0);
        ESP_LOGI(TAG, "FW notifications %s", g_fwNotifyEnabled ? "ENABLED" : "disabled");
    } else if (param->write.handle == g_otaCtrlCharHandle) {
        handleOtaCommand(param->write.value, param->write.len);
    } else if (param->write.handle == g_otaDataCharHandle) {
        handleOtaData(param->write.value, param->write.len);
    }

    if (param->write.need_rsp) {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
    }
}

static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        handleRegEvent(gatts_if);
        break;
    case ESP_GATTS_CREATE_EVT:
        handleCreateEvent(gatts_if, param);
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        handleAddCharEvent(gatts_if, param);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        handleAddDescrEvent(gatts_if, param);
        break;
    case ESP_GATTS_CONNECT_EVT:
        g_connId = param->connect.conn_id;
        g_connected = true;
        // Reset notification state on new connection
        g_otaCtrlNotifyEnabled = false;
        g_fwNotifyEnabled = false;
        ESP_LOGI(TAG, "Connected, conn_id=%d", g_connId);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        g_connected = false;
        ESP_LOGI(TAG, "Disconnected");
        startAdvertising();
        break;
    case ESP_GATTS_READ_EVT:
        handleReadEvent(gatts_if, param);
        break;
    case ESP_GATTS_WRITE_EVT:
        handleWriteEvent(gatts_if, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU: %d", param->mtu.mtu);
        break;
    default:
        break;
    }
}

// ============================================================
// BLE Initialization
// ============================================================
static bool initBle() {
    uuid128FromString(BLE_SERVICE_UUID_CONTROL, g_uuidService);
    uuid128FromString(BLE_CHAR_UUID_OTA_CTRL, g_uuidOtaCtrl);
    uuid128FromString(BLE_CHAR_UUID_OTA_DATA, g_uuidOtaData);
    uuid128FromString(BLE_CHAR_UUID_FWVER, g_uuidFw);
    
    // Initialize Bluetooth controller in BTDM mode
    // (Classic BT is compiled in but we only use BLE for OTA)
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    esp_ble_gatt_set_local_mtu(517);
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gattsEventHandler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    ESP_LOGI(TAG, "BLE initialized (Bluedroid)");
    return true;
}

// ============================================================
// Main
// ============================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, " ESP32 Recovery Mode (Bluedroid)");
    ESP_LOGI(TAG, " Version: %s", RECOVERY_VERSION);
    ESP_LOGI(TAG, "===================================");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(TAG, "Running: %s @ 0x%08" PRIx32, running->label, (uint32_t)running->address);
    }
    
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    if (next) {
        ESP_LOGI(TAG, "OTA target: %s (%u KB)", next->label, (unsigned)(next->size / 1024));
    }
    
    if (!initBle()) {
        ESP_LOGE(TAG, "BLE init failed!");
        return;
    }
    
    ESP_LOGI(TAG, "Ready - connect to '%s'", DEVICE_NAME);
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
