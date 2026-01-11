#!/usr/bin/env pwsh
# Flash firmware to BOTH factory and ota_0 partitions
# Usage: .\flash_both.ps1 [-Port COM10]

param(
    [string]$Port = "COM10"
)

$ErrorActionPreference = "Stop"

# Partition addresses from partitions.csv
$FACTORY_ADDR = "0x10000"
$OTA0_ADDR = "0x310000"

$PROJECT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$BUILD_DIR = Join-Path $PROJECT_DIR "build"
$BIN_FILE = Join-Path $BUILD_DIR "esp32-a2dp-sink.bin"

# Check if binary exists
if (-not (Test-Path $BIN_FILE)) {
    Write-Host "Error: Firmware binary not found at $BIN_FILE" -ForegroundColor Red
    Write-Host "Run 'idf.py build' first." -ForegroundColor Yellow
    exit 1
}

Write-Host "======================================" -ForegroundColor Cyan
Write-Host " Flashing to BOTH partitions" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "Binary: $BIN_FILE"
Write-Host "Port: $Port"
Write-Host ""

# Flash bootloader, partition table, and factory app normally
Write-Host "[1/2] Flashing bootloader + partition table + factory..." -ForegroundColor Green
& idf.py -p $Port flash
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Flash failed!" -ForegroundColor Red
    exit 1
}

# Also write the app binary to ota_0 partition
Write-Host ""
Write-Host "[2/2] Writing firmware to ota_0 ($OTA0_ADDR)..." -ForegroundColor Green
& esptool.py -p $Port -b 460800 write_flash $OTA0_ADDR $BIN_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: ota_0 flash failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "======================================" -ForegroundColor Green
Write-Host " SUCCESS: Both partitions flashed!" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host "  factory @ $FACTORY_ADDR"
Write-Host "  ota_0   @ $OTA0_ADDR"
Write-Host ""
Write-Host "Run 'idf.py -p $Port monitor' to view output."
