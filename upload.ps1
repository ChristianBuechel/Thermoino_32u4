# --- CONFIG ---
$scriptDir      = $PSScriptRoot
$deviceName     = "Pololu A-Star 32u4"
$bootloaderName = "Pololu A-Star 32u4 bootloader"

$firmwarePath   = Join-Path $scriptDir "firmware.hex"
$avrdudeConfig  = Join-Path $scriptDir "avrdude.conf"
$avrdudeExe     = Join-Path $scriptDir "avrdude.exe"

function Get-ComPortByCaption($captionPattern) {
    $dev = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Caption -like "*$captionPattern*" }

    if (-not $dev) { return $null }

    if ($dev.Caption -match "\(COM(\d+)\)") {
        return "COM$($matches[1])"
    }

    return $null
}

# --- Check required files ---
if (-not (Test-Path $firmwarePath)) {
    Write-Host "ERROR: Firmware file not found: $firmwarePath"
    exit 1
}

if (-not (Test-Path $avrdudeConfig)) {
    Write-Host "ERROR: avrdude.conf not found: $avrdudeConfig"
    exit 1
}

if (-not (Test-Path $avrdudeExe)) {
    Write-Host "ERROR: avrdude.exe not found: $avrdudeExe"
    exit 1
}

# --- Find normal Leonardo ---
Write-Host "Searching for $deviceName..."
$normalPort = Get-ComPortByCaption $deviceName

if (-not $normalPort) {
    Write-Host "ERROR: Device not found."
    exit 1
}

Write-Host "Found device on $normalPort"

# --- Trigger bootloader ---
Write-Host "Triggering bootloader..."
$port = new-Object System.IO.Ports.SerialPort $normalPort,1200,None,8,one
$port.Open()
Start-Sleep -Milliseconds 100
$port.Close()

# --- Wait for bootloader ---
Write-Host "Waiting for bootloader..."
$bootPort = $null
$timeout = [DateTime]::Now.AddSeconds(10)

while (-not $bootPort -and [DateTime]::Now -lt $timeout) {
    Start-Sleep -Milliseconds 200
    $bootPort = Get-ComPortByCaption $bootloaderName
}

if (-not $bootPort) {
    Write-Host "ERROR: Bootloader did not appear."
    exit 1
}

Write-Host "Bootloader detected on $bootPort"



Write-Host "DEBUG:"
Write-Host "  avrdude.exe   = '$avrdudeExe'"
Write-Host "  avrdude.conf  = '$avrdudeConfig'"
Write-Host "  firmware.hex  = '$firmwarePath'"
Write-Host "  Port          = '$bootPort'"
Write-Host "  Test          = 'flash:w:""$firmwarePath"":i'"
Write-Host ""

# --- Flash firmware ---
Write-Host "Flashing firmware..."
& $avrdudeExe `
    -v `
    -C "$avrdudeConfig" `
    -patmega32u4 `
    -cavr109 `
    -P "$bootPort" `
    -b57600 `
    -D `
    -V `
    -U "flash:w:""$firmwarePath"":i"

Write-Host "Done."