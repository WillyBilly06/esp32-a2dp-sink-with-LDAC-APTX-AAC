# Build and Flash Recovery + Main Firmware
# This script builds both the recovery (factory) partition and the main app,
# then flashes both to the device.

param(
    [switch]$BuildOnly,
    [switch]$RecoveryOnly,
    [switch]$MainOnly,
    [string]$Port = "COM10"
)

$ErrorActionPreference = "Stop"

# Source ESP-IDF environment
Write-Host "Setting up ESP-IDF environment..." -ForegroundColor Cyan
. D:\esp-idf\export.ps1

$MainDir = "D:\esp32-a2dp-sink"
$RecoveryDir = "D:\esp32-a2dp-sink\recovery"

# Build Recovery partition
if (-not $MainOnly) {
    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "Building RECOVERY partition (factory)..." -ForegroundColor Green
    Write-Host "========================================`n" -ForegroundColor Green
    
    Push-Location $RecoveryDir
    try {
        idf.py build
        if ($LASTEXITCODE -ne 0) { throw "Recovery build failed" }
    } finally {
        Pop-Location
    }
    
    Write-Host "`nRecovery build complete!" -ForegroundColor Green
}

# Build Main app
if (-not $RecoveryOnly) {
    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "Building MAIN application..." -ForegroundColor Green
    Write-Host "========================================`n" -ForegroundColor Green
    
    Push-Location $MainDir
    try {
        idf.py build
        if ($LASTEXITCODE -ne 0) { throw "Main build failed" }
    } finally {
        Pop-Location
    }
    
    Write-Host "`nMain build complete!" -ForegroundColor Green
}

# Flash both partitions
if (-not $BuildOnly) {
    Write-Host "`n========================================" -ForegroundColor Yellow
    Write-Host "Flashing all partitions..." -ForegroundColor Yellow
    Write-Host "========================================`n" -ForegroundColor Yellow
    
    # Get partition addresses from partition table
    # factory: 0x10000 (256KB)
    # ota_0:   0x50000
    
    $RecoveryBin = "$RecoveryDir\build\recovery.bin"
    $MainBin = "$MainDir\build\app-template.bin"
    $BootloaderBin = "$MainDir\build\bootloader\bootloader.bin"
    $PartitionTableBin = "$MainDir\build\partition_table\partition-table.bin"
    $OtaDataBin = "$MainDir\build\ota_data_initial.bin"
    
    # Check files exist
    if (-not (Test-Path $RecoveryBin)) {
        Write-Host "Recovery binary not found: $RecoveryBin" -ForegroundColor Red
        Write-Host "Build recovery first with: .\build_recovery.ps1 -RecoveryOnly" -ForegroundColor Yellow
        exit 1
    }
    
    if (-not (Test-Path $MainBin)) {
        Write-Host "Main binary not found: $MainBin" -ForegroundColor Red
        Write-Host "Build main first with: .\build_recovery.ps1 -MainOnly" -ForegroundColor Yellow
        exit 1
    }
    
    Write-Host "Flashing:" -ForegroundColor Cyan
    Write-Host "  Bootloader:      0x1000  <- $BootloaderBin" -ForegroundColor Gray
    Write-Host "  Partition Table: 0x8000  <- $PartitionTableBin" -ForegroundColor Gray
    Write-Host "  Factory/Recovery:0x10000 <- $RecoveryBin" -ForegroundColor Gray
    Write-Host "  OTA_0 (Main):    0x50000 <- $MainBin" -ForegroundColor Gray
    Write-Host "  OTA Data:        0x610000<- $OtaDataBin" -ForegroundColor Gray
    Write-Host ""
    
    # Flash all at once
    esptool.py --chip esp32 -p $Port -b 460800 `
        --before=default_reset --after=hard_reset write_flash `
        --flash_mode dio --flash_freq 80m --flash_size 8MB `
        0x1000 $BootloaderBin `
        0x8000 $PartitionTableBin `
        0x10000 $RecoveryBin `
        0x50000 $MainBin `
        0x610000 $OtaDataBin
    
    if ($LASTEXITCODE -ne 0) { throw "Flash failed" }
    
    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "Flash complete!" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Normal boot: Device will boot into main app (ota_0)" -ForegroundColor Cyan
    Write-Host "Recovery:    Hold Button 1 during boot to enter recovery mode" -ForegroundColor Yellow
    Write-Host ""
}
