/**
 * @file recovery_nimble.c
 * @brief BLE OTA Recovery firmware using NimBLE stack
 * 
 * This is a minimal recovery firmware that provides BLE-based OTA updates.
 * It uses the NimBLE stack (smaller than Bluedroid) to fit in the recovery partition.
 * 
 * GATT Service:
 *   UUID: 12345678-1234-1234-1234-1234567890ad
 * 
 * Characteristics:
 *   - OTA Control (b1): Write + Notify - for commands (BEGIN, CHECK, END, ABORT)
 *   - OTA Data   (b2): Write + Write No Response - for firmware binary data
 *   - FW Version (b3): Read - returns firmware version string
 */

#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char* TAG = "RECOVERY";

// -----------------------------------------------------------
// BLE UUIDs (must match Android app)
// -----------------------------------------------------------
// Service UUID: 12345678-1234-1234-1234-1234567890ad
static const ble_uuid128_t SERVICE_UUID = BLE_UUID128_INIT(
    0xad, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

// OTA Control: 12345678-1234-1234-1234-1234567890b1
static const ble_uuid128_t OTA_CTRL_UUID = BLE_UUID128_INIT(
    0xb1, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

// OTA Data: 12345678-1234-1234-1234-1234567890b2
static const ble_uuid128_t OTA_DATA_UUID = BLE_UUID128_INIT(
    0xb2, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

// FW Version: 12345678-1234-1234-1234-1234567890b3
static const ble_uuid128_t FW_VERSION_UUID = BLE_UUID128_INIT(
    0xb3, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

// -----------------------------------------------------------
// OTA State
// -----------------------------------------------------------
static bool              g_otaActive = false;
static uint32_t          g_otaReceived = 0;
static uint32_t          g_otaTotalSize = 0;
static int64_t           g_otaCheckPassedTime = 0;
static esp_ota_handle_t  g_otaHandle = 0;
static const esp_partition_t* g_otaPartition = NULL;

// -----------------------------------------------------------
// BLE State
// -----------------------------------------------------------
static uint16_t g_connHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_otaCtrlValHandle = 0;
static bool     g_otaCtrlNotifyEnabled = false;
static uint8_t  g_ownAddrType = BLE_OWN_ADDR_PUBLIC;

// Firmware version
static const char* RECOVERY_VERSION = "RECOVERY v1.0";

// -----------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------
static void startAdvertising();
static int gapEventHandler(struct ble_gap_event* event, void* arg);
static int otaCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg);
static int otaDataAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg);
static int fwVersionAccess(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt* ctxt, void* arg);

// -----------------------------------------------------------
// GATT Service Definition
// -----------------------------------------------------------
static const struct ble_gatt_svc_def gattServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // OTA Control - Write + Notify
                .uuid = &OTA_CTRL_UUID.u,
                .access_cb = otaCtrlAccess,
                .val_handle = &g_otaCtrlValHandle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // OTA Data - Write + Write No Response
                .uuid = &OTA_DATA_UUID.u,
                .access_cb = otaDataAccess,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // FW Version - Read only
                .uuid = &FW_VERSION_UUID.u,
                .access_cb = fwVersionAccess,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 } // Terminator
        },
    },
    { 0 } // Service terminator
};

// -----------------------------------------------------------
// Send notification to client
// -----------------------------------------------------------
static void sendNotification(const char* msg)
{
    if (g_connHandle == BLE_HS_CONN_HANDLE_NONE || !g_otaCtrlNotifyEnabled) {
        ESP_LOGW(TAG, "Cannot send notification - not connected or not enabled");
        return;
    }
    
    struct os_mbuf* om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (om) {
        int rc = ble_gattc_notify_custom(g_connHandle, g_otaCtrlValHandle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Notify failed: %d", rc);
        }
    }
}

// -----------------------------------------------------------
// OTA Finalize Task
// -----------------------------------------------------------
static void otaFinalizeTask(void* arg)
{
    // Wait 3 seconds for potential ABORT
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Check if still valid
    if (!g_otaActive || g_otaHandle == 0) {
        ESP_LOGW(TAG, "OTA already aborted or not active");
        vTaskDelete(NULL);
        return;
    }
    
    int64_t now = esp_timer_get_time();
    if (g_otaCheckPassedTime == 0 || (now - g_otaCheckPassedTime) < 2000000) {
        ESP_LOGW(TAG, "CHECK not passed or too soon");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Auto-finalizing OTA...");
    
    esp_err_t err = esp_ota_end(g_otaHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        sendNotification("FAIL:OTA_END_ERROR");
        g_otaActive = false;
        g_otaHandle = 0;
        vTaskDelete(NULL);
        return;
    }
    
    err = esp_ota_set_boot_partition(g_otaPartition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        sendNotification("FAIL:SET_BOOT_ERROR");
        g_otaActive = false;
        g_otaHandle = 0;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "OTA complete! Rebooting in 2 seconds...");
    sendNotification("DONE");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

// -----------------------------------------------------------
// Handle OTA Control Commands
// -----------------------------------------------------------
static void handleOtaCtrl(const uint8_t* data, size_t len)
{
    if (len == 0) return;
    
    // Null-terminate for string parsing
    char cmd[256];
    size_t cmdLen = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
    memcpy(cmd, data, cmdLen);
    cmd[cmdLen] = '\0';
    
    ESP_LOGI(TAG, "OTA Ctrl command: %s", cmd);
    
    // BEGIN:<size> - Start OTA
    if (strncmp(cmd, "BEGIN:", 6) == 0) {
        uint32_t size = (uint32_t)atoi(cmd + 6);
        ESP_LOGI(TAG, "OTA BEGIN, size=%lu", (unsigned long)size);
        
        if (g_otaActive) {
            ESP_LOGW(TAG, "OTA already active, aborting previous");
            if (g_otaHandle) {
                esp_ota_abort(g_otaHandle);
                g_otaHandle = 0;
            }
        }
        
        // Find OTA partition (ota_0 or ota_1)
        g_otaPartition = esp_ota_get_next_update_partition(NULL);
        if (!g_otaPartition) {
            ESP_LOGE(TAG, "No OTA partition found");
            sendNotification("FAIL:NO_OTA_PARTITION");
            return;
        }
        
        ESP_LOGI(TAG, "OTA target partition: %s at 0x%lx, size %lu",
                 g_otaPartition->label, 
                 (unsigned long)g_otaPartition->address,
                 (unsigned long)g_otaPartition->size);
        
        if (size > g_otaPartition->size) {
            ESP_LOGE(TAG, "Firmware too large for partition");
            sendNotification("FAIL:FW_TOO_LARGE");
            return;
        }
        
        esp_err_t err = esp_ota_begin(g_otaPartition, OTA_WITH_SEQUENTIAL_WRITES, &g_otaHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
            sendNotification("FAIL:OTA_BEGIN_ERROR");
            return;
        }
        
        g_otaTotalSize = size;
        g_otaReceived = 0;
        g_otaActive = true;
        g_otaCheckPassedTime = 0;
        
        sendNotification("READY");
        ESP_LOGI(TAG, "OTA started, waiting for data...");
    }
    // CHECK:<size> - Verify received size
    else if (strncmp(cmd, "CHECK:", 6) == 0) {
        uint32_t expectedSize = (uint32_t)atoi(cmd + 6);
        ESP_LOGI(TAG, "OTA CHECK: expected=%lu, received=%lu", 
                 (unsigned long)expectedSize, (unsigned long)g_otaReceived);
        
        if (!g_otaActive) {
            sendNotification("CHECK_FAIL:NOT_ACTIVE");
            return;
        }
        
        if (g_otaReceived == expectedSize) {
            g_otaCheckPassedTime = esp_timer_get_time();
            char resp[64];
            snprintf(resp, sizeof(resp), "CHECK_OK:%" PRIu32, g_otaReceived);
            sendNotification(resp);
            
            // Start auto-finalize task
            xTaskCreate(otaFinalizeTask, "ota_finalize", 4096, NULL, 5, NULL);
        } else {
            char resp[64];
            snprintf(resp, sizeof(resp), "CHECK_FAIL:%" PRIu32, g_otaReceived);
            sendNotification(resp);
        }
    }
    // END - Explicit finalize (optional, auto-finalize preferred)
    else if (strcmp(cmd, "END") == 0) {
        ESP_LOGI(TAG, "OTA END command");
        
        if (!g_otaActive || g_otaHandle == 0) {
            sendNotification("FAIL:NOT_ACTIVE");
            return;
        }
        
        esp_err_t err = esp_ota_end(g_otaHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            sendNotification("FAIL:OTA_END_ERROR");
            g_otaActive = false;
            g_otaHandle = 0;
            return;
        }
        
        err = esp_ota_set_boot_partition(g_otaPartition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
            sendNotification("FAIL:SET_BOOT_ERROR");
            g_otaActive = false;
            g_otaHandle = 0;
            return;
        }
        
        sendNotification("DONE");
        ESP_LOGI(TAG, "OTA complete! Rebooting...");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    // ABORT - Cancel OTA
    else if (strcmp(cmd, "ABORT") == 0) {
        ESP_LOGW(TAG, "OTA ABORT command");
        
        if (g_otaActive && g_otaHandle) {
            esp_ota_abort(g_otaHandle);
        }
        
        g_otaActive = false;
        g_otaHandle = 0;
        g_otaReceived = 0;
        g_otaTotalSize = 0;
        g_otaCheckPassedTime = 0;
        
        sendNotification("ABORTED");
    }
    else {
        ESP_LOGW(TAG, "Unknown OTA command: %s", cmd);
    }
}

// -----------------------------------------------------------
// Handle OTA Data
// -----------------------------------------------------------
static void handleOtaData(const uint8_t* data, size_t len)
{
    if (!g_otaActive || g_otaHandle == 0) {
        // Silently ignore data when not in OTA mode
        return;
    }
    
    esp_err_t err = esp_ota_write(g_otaHandle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        return;
    }
    
    g_otaReceived += len;
    
    // Log progress every 64KB
    static uint32_t lastLogAt = 0;
    if (g_otaReceived - lastLogAt >= 65536) {
        lastLogAt = g_otaReceived;
        ESP_LOGI(TAG, "OTA progress: %lu / %lu bytes (%lu%%)",
                 (unsigned long)g_otaReceived,
                 (unsigned long)g_otaTotalSize,
                 (unsigned long)(g_otaTotalSize > 0 ? (g_otaReceived * 100 / g_otaTotalSize) : 0));
    }
}

// -----------------------------------------------------------
// GATT Access Callbacks
// -----------------------------------------------------------
static int otaCtrlAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        handleOtaCtrl(ctxt->om->om_data, ctxt->om->om_len);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int otaDataAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        handleOtaData(ctxt->om->om_data, ctxt->om->om_len);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int fwVersionAccess(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, RECOVERY_VERSION, strlen(RECOVERY_VERSION));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// -----------------------------------------------------------
// Start BLE Advertising
// -----------------------------------------------------------
static void startAdvertising()
{
    struct ble_gap_adv_params advParams = {};
    advParams.conn_mode = BLE_GAP_CONN_MODE_UND;
    advParams.disc_mode = BLE_GAP_DISC_MODE_GEN;
    advParams.itvl_min = 0x20;  // 20ms
    advParams.itvl_max = 0x40;  // 40ms
    
    // Advertisement data - includes UUID
    struct ble_hs_adv_fields advFields = {};
    advFields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advFields.uuids128 = &SERVICE_UUID;
    advFields.num_uuids128 = 1;
    advFields.uuids128_is_complete = 1;
    
    int rc = ble_gap_adv_set_fields(&advFields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set adv fields: %d", rc);
        return;
    }
    
    // Scan response - includes device name
    struct ble_hs_adv_fields rspFields = {};
    const char* deviceName = "ESP32-Recovery";
    rspFields.name = (uint8_t*)deviceName;
    rspFields.name_len = strlen(deviceName);
    rspFields.name_is_complete = 1;
    
    rc = ble_gap_adv_rsp_set_fields(&rspFields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response: %d", rc);
        return;
    }
    
    rc = ble_gap_adv_start(g_ownAddrType, NULL, BLE_HS_FOREVER, &advParams, gapEventHandler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Advertising started as '%s'", deviceName);
}

// -----------------------------------------------------------
// GAP Event Handler
// -----------------------------------------------------------
static int gapEventHandler(struct ble_gap_event* event, void* arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "BLE connected, conn_handle=%d", event->connect.conn_handle);
                g_connHandle = event->connect.conn_handle;
                
                // Request larger MTU for faster OTA
                ble_gattc_exchange_mtu(g_connHandle, NULL, NULL);
            } else {
                ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
                g_connHandle = BLE_HS_CONN_HANDLE_NONE;
                startAdvertising();
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
            g_connHandle = BLE_HS_CONN_HANDLE_NONE;
            g_otaCtrlNotifyEnabled = false;
            
            // Abort OTA if active
            if (g_otaActive && g_otaHandle) {
                ESP_LOGW(TAG, "Aborting OTA due to disconnect");
                esp_ota_abort(g_otaHandle);
                g_otaActive = false;
                g_otaHandle = 0;
            }
            
            // Restart advertising
            startAdvertising();
            break;
            
        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
            break;
            
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == g_otaCtrlValHandle) {
                g_otaCtrlNotifyEnabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "OTA Ctrl notify %s", g_otaCtrlNotifyEnabled ? "enabled" : "disabled");
            }
            break;
            
        default:
            break;
    }
    
    return 0;
}

// -----------------------------------------------------------
// NimBLE Host Task
// -----------------------------------------------------------
static void bleHostTask(void* arg)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// -----------------------------------------------------------
// BLE Sync Callback
// -----------------------------------------------------------
static void onBleSync()
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
        return;
    }
    
    rc = ble_hs_id_infer_auto(0, &g_ownAddrType);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
        return;
    }
    
    startAdvertising();
}

// -----------------------------------------------------------
// BLE Reset Callback
// -----------------------------------------------------------
static void onBleReset(int reason)
{
    ESP_LOGW(TAG, "BLE reset, reason=%d", reason);
}

// -----------------------------------------------------------
// Initialize NimBLE
// -----------------------------------------------------------
static esp_err_t initBle()
{
    ESP_LOGI(TAG, "Initializing NimBLE...");
    
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure NimBLE host
    ble_hs_cfg.reset_cb = onBleReset;
    ble_hs_cfg.sync_cb = onBleSync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Register our OTA service
    int rc = ble_gatts_count_cfg(gattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }
    
    rc = ble_gatts_add_svcs(gattServices);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }
    
    // Set device name
    rc = ble_svc_gap_device_name_set("ESP32-Recovery");
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to set device name: %d", rc);
    }
    
    // Start BLE host task
    nimble_port_freertos_init(bleHostTask);
    
    ESP_LOGI(TAG, "NimBLE initialized");
    return ESP_OK;
}

// -----------------------------------------------------------
// Main Entry Point
// -----------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, " ESP32 Recovery Mode");
    ESP_LOGI(TAG, " BLE OTA Firmware Update");
    ESP_LOGI(TAG, "=================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Log partition info
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(TAG, "Running from: %s (0x%lx)", 
                 running->label, (unsigned long)running->address);
    }
    
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    if (next) {
        ESP_LOGI(TAG, "OTA target: %s (0x%lx, %lu bytes)",
                 next->label, (unsigned long)next->address, (unsigned long)next->size);
    }
    
    // Initialize BLE
    ret = initBle();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed!");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "Recovery mode ready. Connect via BLE to update firmware.");
    ESP_LOGI(TAG, "Service UUID: 12345678-1234-1234-1234-1234567890ad");
    
    // Main loop - just keep running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
