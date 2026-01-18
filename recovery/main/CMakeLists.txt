# Recovery main component - uses Bluedroid BLE-only (no A2DP)
idf_component_register(
    SRCS "recovery_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_partition app_update esp_timer
)
