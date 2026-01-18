// -----------------------------------------------------------
// Recovery Firmware - Bluedroid BLE OTA-only application
// This is a stripped-down firmware that only supports:
// - BLE advertising (same UUID as main app for app recognition)
// - OTA firmware update via BLE
// Uses Bluedroid stack in BLE-only mode for compatibility with main app
// -----------------------------------------------------------

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

// Bluedroid BLE includes
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

static const char* TAG = "Recovery";

// -----------------------------------------------------------
// BLE UUIDs (must match main app for app recognition)
// -----------------------------------------------------------
// Service UUID: 12345678-1234-1234-1234-1234567890ad
static uint8_t g_serviceUuid[16] = {
    0xad, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

// FW Version: 12345678-1234-1234-1234-1234567890b3
static uint8_t g_uuidFwVer[16] = {
    0xb3, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

// OTA Ctrl: 12345678-1234-1234-1234-1234567890b1
static uint8_t g_uuidOtaCtrl[16] = {
    0xb1, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

// OTA Data: 12345678-1234-1234-1234-1234567890b2
static uint8_t g_uuidOtaData[16] = {
    0xb2, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

// -----------------------------------------------------------
// OTA State
// -----------------------------------------------------------
static bool              g_otaActive = false;
static uint32_t          g_otaReceived = 0;
static uint32_t          g_otaTotalSize = 0;
static int64_t           g_otaCheckPassedTime = 0;
static esp_ota_handle_t  g_otaHandle = 0;
static const esp_partition_t* g_otaPartition = nullptr;

// -----------------------------------------------------------
// BLE State
// -----------------------------------------------------------
static esp_gatt_if_t g_gattsIf = 0;
static uint16_t g_connId = 0;
static bool g_connected = false;
static uint16_t g_serviceHandle = 0;
static uint16_t g_fwVerCharHandle = 0;
static uint16_t g_otaCtrlCharHandle = 0;
static uint16_t g_otaDataCharHandle = 0;
static uint16_t g_otaCtrlCccdHandle = 0;
static bool g_otaCtrlNotifyEnabled = false;

// Device info
static const char* DEVICE_NAME = "BDK RECOVERY";
static const char* FW_VERSION = "RECOVERY 1.0";

// Advertising data
static esp_ble_adv_params_t g_advParams = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr          = {0},
    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// -----------------------------------------------------------
// Notify OTA control status
// -----------------------------------------------------------
static void notifyOtaCtrl(const char* msg) {
    ESP_LOGI(TAG, "Notifying: %s", msg);
    if (g_connected && g_otaCtrlNotifyEnabled && g_otaCtrlCharHandle && g_gattsIf) {
        esp_err_t ret = esp_ble_gatts_send_indicate(g_gattsIf, g_connId, g_otaCtrlCharHandle,
                                    strlen(msg), (uint8_t*)msg, false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send notification: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "Cannot notify: conn=%d, enabled=%d, handle=%d", 
                 g_connected, g_otaCtrlNotifyEnabled, g_otaCtrlCharHandle);
    }
}

// -----------------------------------------------------------
// OTA Finalize
// -----------------------------------------------------------
static void finalizeOta() {
    if (!g_otaActive || !g_otaHandle) {
        ESP_LOGE(TAG, "OTA finalize called but no active OTA");
        notifyOtaCtrl("END_ERR");
        return;
    }
    
    ESP_LOGI(TAG, "Finalizing OTA: received %u / %u bytes", 
             (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
    
    esp_err_t err = esp_ota_end(g_otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        notifyOtaCtrl("END_ERR");
        g_otaActive = false;
        g_otaHandle = 0;
        return;
    }
    
    err = esp_ota_set_boot_partition(g_otaPartition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        notifyOtaCtrl("END_ERR");
        g_otaActive = false;
        g_otaHandle = 0;
        return;
    }
    
    notifyOtaCtrl("END_OK");
    ESP_LOGI(TAG, "OTA complete, restarting in 1s...");
    
    g_otaActive = false;
    g_otaHandle = 0;
    g_otaCheckPassedTime = 0;
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

// -----------------------------------------------------------
// Auto-finalize task
// -----------------------------------------------------------
static void autoFinalizeTask(void* param) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    if (g_otaActive && g_otaCheckPassedTime > 0) {
        ESP_LOGW(TAG, "OTA: No END received after CHECK passed, auto-finalizing...");
        finalizeOta();
    }
    
    vTaskDelete(NULL);
}

// -----------------------------------------------------------
// OTA Control Handler
// -----------------------------------------------------------
static void handleOtaCtrl(const uint8_t* data, size_t len) {
    ESP_LOGI(TAG, "OTA CTRL: len=%u, first=0x%02X '%c'", 
             (unsigned)len, len > 0 ? data[0] : 0, 
             (len > 0 && data[0] >= 32 && data[0] < 127) ? data[0] : '?');
    if (len < 1) return;
    
    // ASCII: BEGIN:<size>
    if (data[0] == 'B' && len >= 6 && memcmp(data, "BEGIN:", 6) == 0) {
        char sizeBuf[16] = {0};
        size_t copyLen = (len - 6 < sizeof(sizeBuf) - 1) ? (len - 6) : (sizeof(sizeBuf) - 1);
        memcpy(sizeBuf, data + 6, copyLen);
        uint32_t size = (uint32_t)atoi(sizeBuf);
        
        ESP_LOGI(TAG, "OTA BEGIN (ASCII): %u bytes", (unsigned)size);
        
        g_otaPartition = esp_ota_get_next_update_partition(NULL);
        if (!g_otaPartition) {
            ESP_LOGE(TAG, "No OTA partition found");
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        ESP_LOGI(TAG, "OTA partition: %s, addr=0x%lx, size=%lu",
                 g_otaPartition->label, (unsigned long)g_otaPartition->address,
                 (unsigned long)g_otaPartition->size);
        
        if (size > g_otaPartition->size) {
            ESP_LOGE(TAG, "Image too large: %u > %lu", 
                     (unsigned)size, (unsigned long)g_otaPartition->size);
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        esp_err_t err = esp_ota_begin(g_otaPartition, size, &g_otaHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        g_otaActive = true;
        g_otaReceived = 0;
        g_otaTotalSize = size;
        g_otaCheckPassedTime = 0;
        
        notifyOtaCtrl("BEGIN_OK");
        ESP_LOGI(TAG, "OTA started, waiting for data...");
        return;
    }
    
    // ASCII: END
    if (data[0] == 'E' && len >= 3 && memcmp(data, "END", 3) == 0) {
        g_otaCheckPassedTime = 0;
        ESP_LOGI(TAG, "OTA END received");
        
        uint32_t lastReceived = g_otaReceived;
        for (int i = 0; i < 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (g_otaReceived == lastReceived) break;
            lastReceived = g_otaReceived;
            ESP_LOGI(TAG, "OTA: still receiving, now at %u bytes", (unsigned)g_otaReceived);
        }
        
        finalizeOta();
        return;
    }
    
    // ASCII: CHECK:<size>
    if (data[0] == 'C' && len >= 6 && memcmp(data, "CHECK:", 6) == 0) {
        uint32_t expectedSize = 0;
        for (size_t i = 6; i < len && data[i] >= '0' && data[i] <= '9'; i++) {
            expectedSize = expectedSize * 10 + (data[i] - '0');
        }
        
        ESP_LOGI(TAG, "OTA CHECK: received %u / %u expected", 
                 (unsigned)g_otaReceived, (unsigned)expectedSize);
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        if (g_otaReceived >= expectedSize) {
            notifyOtaCtrl("CHECK_OK");
            ESP_LOGI(TAG, "OTA CHECK passed");
            g_otaCheckPassedTime = esp_timer_get_time();
            xTaskCreate(autoFinalizeTask, "ota_auto", 4096, NULL, 5, NULL);
        } else {
            char resp[32];
            snprintf(resp, sizeof(resp), "CHECK_FAIL:%u", (unsigned)g_otaReceived);
            notifyOtaCtrl(resp);
            ESP_LOGW(TAG, "OTA CHECK failed: missing %u bytes", 
                     (unsigned)(expectedSize - g_otaReceived));
        }
        return;
    }
    
    // ASCII: ABORT
    if (data[0] == 'A' && len >= 5 && memcmp(data, "ABORT", 5) == 0) {
        ESP_LOGW(TAG, "OTA ABORT requested");
        if (g_otaActive && g_otaHandle) {
            esp_ota_abort(g_otaHandle);
        }
        g_otaActive = false;
        g_otaHandle = 0;
        g_otaReceived = 0;
        g_otaTotalSize = 0;
        g_otaCheckPassedTime = 0;
        notifyOtaCtrl("ABORT_OK");
        return;
    }
    
    // Binary protocol
    uint8_t cmd = data[0];
    
    if (cmd == 0x01 && len >= 5) {
        uint32_t size = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        ESP_LOGI(TAG, "OTA BEGIN (binary): %u bytes", (unsigned)size);
        
        g_otaPartition = esp_ota_get_next_update_partition(NULL);
        if (!g_otaPartition || size > g_otaPartition->size) {
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        esp_err_t err = esp_ota_begin(g_otaPartition, size, &g_otaHandle);
        if (err != ESP_OK) {
            notifyOtaCtrl("BEGIN_ERR");
            return;
        }
        
        g_otaActive = true;
        g_otaReceived = 0;
        g_otaTotalSize = size;
        g_otaCheckPassedTime = 0;
        notifyOtaCtrl("BEGIN_OK");
    } else if (cmd == 0x03) {
        g_otaCheckPassedTime = 0;
        ESP_LOGI(TAG, "OTA END (binary)");
        uint32_t lastReceived = g_otaReceived;
        for (int i = 0; i < 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (g_otaReceived == lastReceived) break;
            lastReceived = g_otaReceived;
        }
        finalizeOta();
    } else if (cmd == 0x04) {
        ESP_LOGW(TAG, "OTA ABORT (binary)");
        if (g_otaActive && g_otaHandle) esp_ota_abort(g_otaHandle);
        g_otaActive = false;
        g_otaHandle = 0;
        g_otaReceived = 0;
        g_otaCheckPassedTime = 0;
        notifyOtaCtrl("ABORT_OK");
    }
}

// -----------------------------------------------------------
// OTA Data Handler
// -----------------------------------------------------------
static void handleOtaData(const uint8_t* data, size_t len) {
    if (!g_otaActive || !g_otaHandle) return;
    if (len == 1) return;  // Ignore flush packets
    
    esp_err_t err = esp_ota_write(g_otaHandle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        return;
    }
    
    g_otaReceived += len;
    
    uint8_t pct = (g_otaTotalSize > 0) ? (uint8_t)((uint64_t)g_otaReceived * 100 / g_otaTotalSize) : 0;
    static uint8_t lastPctLogged = 255;
    if (pct != lastPctLogged && (pct % 5 == 0 || pct == 100)) {
        ESP_LOGI(TAG, "OTA: %3u%% (%u / %u bytes)", pct, (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
        lastPctLogged = pct;
    }
}

// -----------------------------------------------------------
// GATTS Event Handler
// -----------------------------------------------------------
static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t* param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATTS_REG_EVT");
            g_gattsIf = gatts_if;
            
            // Configure advertising data
            esp_ble_adv_data_t adv_data = {};
            adv_data.set_scan_rsp = false;
            adv_data.include_name = false;  // Name in scan response
            adv_data.include_txpower = true;
            adv_data.min_interval = 0x0006;
            adv_data.max_interval = 0x0010;
            adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
            adv_data.service_uuid_len = 16;
            adv_data.p_service_uuid = g_serviceUuid;

            esp_ble_adv_data_t scan_rsp = {};
            scan_rsp.set_scan_rsp = true;
            scan_rsp.include_name = true;
            scan_rsp.include_txpower = true;

            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gap_config_adv_data(&scan_rsp);
            
            // Create service
            esp_gatt_srvc_id_t svc = {};
            svc.is_primary = true;
            svc.id.inst_id = 0x00;
            svc.id.uuid.len = ESP_UUID_LEN_128;
            memcpy(svc.id.uuid.uuid.uuid128, g_serviceUuid, 16);
            // Need 10 handles: 1 svc + (3 chars * 2) + 3 CCCDs = 10
            esp_ble_gatts_create_service(gatts_if, &svc, 12);
            break;
        }
        
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "Service created, handle=%d", param->create.service_handle);
            g_serviceHandle = param->create.service_handle;
            esp_ble_gatts_start_service(g_serviceHandle);
            
            // Add FW Version characteristic (read only)
            static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
            esp_bt_uuid_t uuid = {};
            uuid.len = ESP_UUID_LEN_128;
            memcpy(uuid.uuid.uuid128, g_uuidFwVer, 16);
            esp_attr_value_t val = {};
            val.attr_max_len = 64;
            val.attr_len = 0;
            esp_ble_gatts_add_char(g_serviceHandle, &uuid, ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ, &val, &rspByApp);
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Char added, handle=%d", handle);
            
            // Determine which characteristic was added by checking UUID
            if (memcmp(param->add_char.char_uuid.uuid.uuid128, g_uuidFwVer, 16) == 0) {
                g_fwVerCharHandle = handle;
                
                // Add OTA Ctrl characteristic
                static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
                esp_bt_uuid_t uuid = {};
                uuid.len = ESP_UUID_LEN_128;
                memcpy(uuid.uuid.uuid128, g_uuidOtaCtrl, 16);
                esp_attr_value_t val = {};
                val.attr_max_len = 256;
                esp_ble_gatts_add_char(g_serviceHandle, &uuid, ESP_GATT_PERM_WRITE,
                                       ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       &val, &rspByApp);
            } else if (memcmp(param->add_char.char_uuid.uuid.uuid128, g_uuidOtaCtrl, 16) == 0) {
                g_otaCtrlCharHandle = handle;
                
                // Add CCCD for OTA Ctrl
                esp_bt_uuid_t cccdUuid = {};
                cccdUuid.len = ESP_UUID_LEN_16;
                cccdUuid.uuid.uuid16 = 0x2902;
                static esp_attr_control_t cccdRsp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
                esp_ble_gatts_add_char_descr(g_serviceHandle, &cccdUuid,
                                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, &cccdRsp);
            } else if (memcmp(param->add_char.char_uuid.uuid.uuid128, g_uuidOtaData, 16) == 0) {
                g_otaDataCharHandle = handle;
                ESP_LOGI(TAG, "All characteristics added. FW=%d, OtaCtrl=%d, OtaData=%d",
                         g_fwVerCharHandle, g_otaCtrlCharHandle, g_otaDataCharHandle);
            }
            break;
        }
        
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            g_otaCtrlCccdHandle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD added, handle=%d", g_otaCtrlCccdHandle);
            
            // Add OTA Data characteristic (write only, no response for speed)
            static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
            esp_bt_uuid_t uuid = {};
            uuid.len = ESP_UUID_LEN_128;
            memcpy(uuid.uuid.uuid128, g_uuidOtaData, 16);
            esp_attr_value_t val = {};
            val.attr_max_len = 512;
            esp_ble_gatts_add_char(g_serviceHandle, &uuid, ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                   &val, &rspByApp);
            break;
        }
        
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started");
            break;
            
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(TAG, "BLE connected, conn_id=%d", param->connect.conn_id);
            g_connId = param->connect.conn_id;
            g_connected = true;
            
            // Update connection params for faster OTA
            esp_ble_conn_update_params_t conn_params = {};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x10;   // 20ms
            conn_params.min_int = 0x08;   // 10ms
            conn_params.timeout = 400;     // 4s
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "BLE disconnected");
            g_connected = false;
            g_otaCtrlNotifyEnabled = false;
            
            if (g_otaActive && g_otaHandle) {
                ESP_LOGW(TAG, "Aborting OTA due to disconnect");
                esp_ota_abort(g_otaHandle);
                g_otaActive = false;
                g_otaHandle = 0;
            }
            
            // Restart advertising
            esp_ble_gap_start_advertising(&g_advParams);
            break;
        }
        
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU updated: %d", param->mtu.mtu);
            break;
            
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(TAG, "Read request, handle=%d", param->read.handle);
            esp_gatt_rsp_t rsp = {};
            rsp.attr_value.handle = param->read.handle;
            
            if (param->read.handle == g_fwVerCharHandle) {
                rsp.attr_value.len = strlen(FW_VERSION);
                memcpy(rsp.attr_value.value, FW_VERSION, rsp.attr_value.len);
            } else if (param->read.handle == g_otaCtrlCharHandle) {
                const char* status = g_otaActive ? "ACTIVE" : "IDLE";
                rsp.attr_value.len = strlen(status);
                memcpy(rsp.attr_value.value, status, rsp.attr_value.len);
            } else if (param->read.handle == g_otaCtrlCccdHandle) {
                rsp.attr_value.len = 2;
                rsp.attr_value.value[0] = g_otaCtrlNotifyEnabled ? 0x01 : 0x00;
                rsp.attr_value.value[1] = 0x00;
            }
            
            esp_ble_gatts_send_response(g_gattsIf, param->read.conn_id, 
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        
        case ESP_GATTS_WRITE_EVT: {
            uint16_t handle = param->write.handle;
            uint8_t* data = param->write.value;
            uint16_t len = param->write.len;
            
            ESP_LOGI(TAG, "Write request, handle=%d, len=%d, need_rsp=%d", 
                     handle, len, param->write.need_rsp);
            
            if (handle == g_otaCtrlCharHandle) {
                handleOtaCtrl(data, len);
            } else if (handle == g_otaDataCharHandle) {
                handleOtaData(data, len);
            } else if (handle == g_otaCtrlCccdHandle && len == 2) {
                g_otaCtrlNotifyEnabled = (data[0] == 0x01);
                ESP_LOGI(TAG, "OTA Ctrl notifications %s", 
                         g_otaCtrlNotifyEnabled ? "enabled" : "disabled");
            }
            
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(g_gattsIf, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        }
        
        case ESP_GATTS_CONF_EVT:
            // Notification confirmed
            break;
            
        default:
            break;
    }
}

// -----------------------------------------------------------
// GAP Event Handler
// -----------------------------------------------------------
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Adv data set complete");
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: 0x%x", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started as '%s'", DEVICE_NAME);
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection params updated");
            break;
            
        default:
            break;
    }
}

// -----------------------------------------------------------
// Initialize BLE
// -----------------------------------------------------------
static esp_err_t initBle() {
    ESP_LOGI(TAG, "Initializing Bluedroid BLE...");
    
    // Init Bluetooth controller in BLE-only mode
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set maximum MTU for faster OTA transfers
    ret = esp_ble_gatt_set_local_mtu(517);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set MTU: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BLE MTU set to 517 bytes");
    }
    
    // Register callbacks
    ret = esp_ble_gap_register_callback(gapEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gatts_register_callback(gattsEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gatts_app_register(0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Bluedroid BLE initialized");
    return ESP_OK;
}

// -----------------------------------------------------------
// Main Entry Point
// -----------------------------------------------------------
extern "C" void app_main() {
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, " BDK RECOVERY MODE (Bluedroid)");
    ESP_LOGI(TAG, " Version: %s", FW_VERSION);
    ESP_LOGI(TAG, "======================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Log partition info
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(TAG, "Running from: %s (0x%lx)", running->label, (unsigned long)running->address);
    }
    
    const esp_partition_t* nextOta = esp_ota_get_next_update_partition(NULL);
    if (nextOta) {
        ESP_LOGI(TAG, "Next OTA target: %s (0x%lx, size=%luKB)", 
                 nextOta->label, (unsigned long)nextOta->address,
                 (unsigned long)(nextOta->size / 1024));
    }
    
    // Initialize BLE
    ret = initBle();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }
    
    // Start advertising after init is complete
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_ble_gap_start_advertising(&g_advParams);
    
    ESP_LOGI(TAG, "Recovery mode ready - connect via BLE to update firmware");
    
    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Recovery idle... OTA %s, received %lu bytes",
                 g_otaActive ? "active" : "inactive",
                 (unsigned long)g_otaReceived);
    }
}
