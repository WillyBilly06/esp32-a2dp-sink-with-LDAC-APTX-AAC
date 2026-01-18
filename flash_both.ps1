#!/usr/bin/env pwsh
# Flash recovery firmware to factory partition and main app to ota_0
# Recovery partition: Minimal BLE OTA-only firmware for emergency recovery
# Main app partition: Full featured app with A2DP + BLE
# Usage: .\flash_both.ps1 [-Port COM10]

param(
    [string]$Port = "COM10"
)

$ErrorActionPreference = "Stop"

# Partition addresses from partitions.csv (updated layout)
$BOOTLOADER_ADDR = "0x1000"
$PARTITION_TABLE_ADDR = "0x8000"
$RECOVERY_ADDR = "0x10000"     # 640KB - Recovery firmware (BLE OTA only, test subtype)
$OTA0_ADDR = "0xB0000"         # 2752KB - Main app
$OTADATA_ADDR = "0x610000"     # 8KB - OTA data

$PROJECT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$BUILD_DIR = Join-Path $PROJECT_DIR "build"
$RECOVERY_DIR = Join-Path $PROJECT_DIR "recovery"
$RECOVERY_BUILD_DIR = Join-Path $RECOVERY_DIR "build"

# Binary files
$BOOTLOADER_BIN = Join-Path $BUILD_DIR "bootloader\bootloader.bin"
$PARTITION_TABLE_BIN = Join-Path $BUILD_DIR "partition_table\partition-table.bin"
$MAIN_APP_BIN = Join-Path $BUILD_DIR "app-template.bin"
$RECOVERY_BIN = Join-Path $RECOVERY_BUILD_DIR "recovery.bin"
$OTADATA_BIN = Join-Path $BUILD_DIR "ota_data_initial.bin"

# Check if all binaries exist
$missing = @()
if (-not (Test-Path $BOOTLOADER_BIN)) { $missing += "Bootloader: $BOOTLOADER_BIN" }
if (-not (Test-Path $PARTITION_TABLE_BIN)) { $missing += "Partition table: $PARTITION_TABLE_BIN" }
if (-not (Test-Path $MAIN_APP_BIN)) { $missing += "Main app: $MAIN_APP_BIN" }
if (-not (Test-Path $RECOVERY_BIN)) { $missing += "Recovery firmware: $RECOVERY_BIN" }

if ($missing.Count -gt 0) {
    Write-Host "Error: Missing binaries:" -ForegroundColor Red
    $missing | ForEach-Object { Write-Host "  - $_" -ForegroundColor Red }
    Write-Host ""
    Write-Host "Build both projects first:" -ForegroundColor Yellow
    Write-Host "  cd recovery && idf.py build && cd .." -ForegroundColor Yellow
    Write-Host "  idf.py build" -ForegroundColor Yellow
    exit 1
}

Write-Host "======================================" -ForegroundColor Cyan
Write-Host " Flashing Recovery + Main App" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "Port: $Port"
Write-Host ""
Write-Host "Partition Layout:" -ForegroundColor Yellow
Write-Host "  Bootloader   @ $BOOTLOADER_ADDR"
Write-Host "  Part. Table  @ $PARTITION_TABLE_ADDR"
Write-Host "  Recovery     @ $RECOVERY_ADDR (recovery partition, test subtype)"
Write-Host "  Main App     @ $OTA0_ADDR (ota_0 partition)"
Write-Host "  OTA Data     @ $OTADATA_ADDR"
Write-Host ""

# Flash all partitions in one command
Write-Host "Flashing all partitions..." -ForegroundColor Green
& esptool.py -p $Port -b 460800 --before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size 8MB --flash_freq 80m `
    $BOOTLOADER_ADDR $BOOTLOADER_BIN `
    $PARTITION_TABLE_ADDR $PARTITION_TABLE_BIN `
    $RECOVERY_ADDR $RECOVERY_BIN `
    $OTA0_ADDR $MAIN_APP_BIN `
    $OTADATA_ADDR $OTADATA_BIN

if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Flash failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "======================================" -ForegroundColor Green
Write-Host " SUCCESS: All partitions flashed!" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host "  Recovery @ $FACTORY_ADDR (factory - BLE OTA only)"
Write-Host "  Main App @ $OTA0_ADDR (ota_0 - full app)"
Write-Host ""
Write-Host "Normal boot: Device runs main app from ota_0"
Write-Host "Recovery:    Hold Button 1 at power-on for 500ms"
Write-Host ""
Write-Host "Run 'idf.py -p $Port monitor' to view output."
