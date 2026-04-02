param(
    [ValidateSet("Release", "Debug")]
    [string]$Configuration = "Release",
    [string]$ToolchainBin = ""
)

$ErrorActionPreference = "Stop"

function Find-ToolchainBin {
    param(
        [string]$ExplicitPath
    )

    if ($ExplicitPath) {
        if (-not (Test-Path -LiteralPath $ExplicitPath)) {
            throw "Specified toolchain bin path does not exist: $ExplicitPath"
        }
        return (Resolve-Path -LiteralPath $ExplicitPath).Path
    }

    $command = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($command) {
        return Split-Path -Parent $command.Source
    }

    $candidates = @(
        "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\bin",
        "C:\Program Files\Arm GNU Toolchain arm-none-eabi\bin",
        (Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages")
    )

    foreach ($candidate in $candidates) {
        if (-not (Test-Path -LiteralPath $candidate)) {
            continue
        }

        $match = Get-ChildItem -LiteralPath $candidate -Recurse -Filter arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($match) {
            return $match.Directory.FullName
        }
    }

    throw "Unable to locate arm-none-eabi-gcc.exe. Install Arm.GnuArmEmbeddedToolchain or pass -ToolchainBin."
}

function Invoke-Checked {
    param(
        [string[]]$CommandLine
    )

    & $CommandLine[0] $CommandLine[1..($CommandLine.Length - 1)]
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $($CommandLine -join ' ')"
    }
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$projectDir = Join-Path $repoRoot "firmware\h743_tof_usb_bridge_cubeide"
$buildRoot = Join-Path $projectDir ("build\" + $Configuration)
$objectRoot = Join-Path $buildRoot "obj"
$artifactBase = "stm32h743-4tof-safety-bridge"
$elfPath = Join-Path $buildRoot ($artifactBase + ".elf")
$hexPath = Join-Path $buildRoot ($artifactBase + ".hex")
$mapPath = Join-Path $buildRoot ($artifactBase + ".map")
$releaseHexPath = Join-Path $repoRoot "release\stm32h743-4tof-safety-bridge.hex"
$linkerScript = Join-Path $projectDir "STM32H743ZITX_FLASH.ld"

$toolchainBinPath = Find-ToolchainBin -ExplicitPath $ToolchainBin
$gcc = Join-Path $toolchainBinPath "arm-none-eabi-gcc.exe"
$objcopy = Join-Path $toolchainBinPath "arm-none-eabi-objcopy.exe"
$size = Join-Path $toolchainBinPath "arm-none-eabi-size.exe"

foreach ($tool in @($gcc, $objcopy, $size)) {
    if (-not (Test-Path -LiteralPath $tool)) {
        throw "Required tool not found: $tool"
    }
}

New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
New-Item -ItemType Directory -Force -Path $objectRoot | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $releaseHexPath) | Out-Null

$includeDirs = @(
    "Core\Inc",
    "Drivers\User\Inc",
    "Drivers\CMSIS\Include",
    "Drivers\CMSIS\Device\ST\STM32H7xx\Include",
    "Drivers\STM32H7xx_HAL_Driver\Inc",
    "USB_DEVICE\App",
    "USB_DEVICE\Target",
    "Middlewares\ST\STM32_USB_Device_Library\Core\Inc",
    "Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc"
) | ForEach-Object { Join-Path $projectDir $_ }

$sourceDirs = @(
    "Core\Src",
    "Drivers\User\Src",
    "USB_DEVICE\App",
    "USB_DEVICE\Target",
    "Drivers\STM32H7xx_HAL_Driver\Src",
    "Middlewares\ST\STM32_USB_Device_Library\Core\Src",
    "Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src"
) | ForEach-Object { Join-Path $projectDir $_ }

$startupSources = Get-ChildItem -LiteralPath (Join-Path $projectDir "Startup") -Filter *.s | Sort-Object FullName
$cSources = foreach ($dir in $sourceDirs) {
    Get-ChildItem -LiteralPath $dir -Filter *.c | Sort-Object FullName
}

$commonFlags = @(
    "-mcpu=cortex-m7",
    "-mthumb",
    "-mfpu=fpv5-d16",
    "-mfloat-abi=hard",
    "-ffunction-sections",
    "-fdata-sections",
    "-fmessage-length=0",
    "-Wall"
)

$configFlags = if ($Configuration -eq "Debug") {
    @("-Og", "-g3", "-DDEBUG")
}
else {
    @("-Os", "-g0")
}

$defineFlags = @(
    "-DUSE_HAL_DRIVER",
    "-DUSE_PWR_LDO_SUPPLY",
    "-DUSE_USB_FS",
    "-DSTM32H743xx"
)

$includeFlags = foreach ($includeDir in $includeDirs) {
    "-I$includeDir"
}

$objects = @()

foreach ($source in $cSources) {
    $relativePath = $source.FullName.Substring($projectDir.Length + 1)
    $objectPath = Join-Path $objectRoot $relativePath
    $objectPath = [System.IO.Path]::ChangeExtension($objectPath, ".o")
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $objectPath) | Out-Null

    $command = @($gcc) + $commonFlags + $configFlags + $defineFlags + $includeFlags + @(
        "-std=gnu11",
        "-c",
        $source.FullName,
        "-o",
        $objectPath
    )
    Invoke-Checked -CommandLine $command
    $objects += $objectPath
}

foreach ($source in $startupSources) {
    $relativePath = $source.FullName.Substring($projectDir.Length + 1)
    $objectPath = Join-Path $objectRoot $relativePath
    $objectPath = [System.IO.Path]::ChangeExtension($objectPath, ".o")
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $objectPath) | Out-Null

    $command = @($gcc) + $commonFlags + $configFlags + $defineFlags + $includeFlags + @(
        "-x",
        "assembler-with-cpp",
        "-c",
        $source.FullName,
        "-o",
        $objectPath
    )
    Invoke-Checked -CommandLine $command
    $objects += $objectPath
}

$linkCommand = @($gcc) + $commonFlags + $configFlags + @(
    "-T",
    $linkerScript,
    "--specs=nosys.specs",
    "--specs=nano.specs",
    "-Wl,-Map=$mapPath",
    "-Wl,--gc-sections",
    "-Wl,--print-memory-usage"
) + $objects + @(
    "-Wl,--start-group",
    "-lc",
    "-lm",
    "-Wl,--end-group",
    "-o",
    $elfPath
)
Invoke-Checked -CommandLine $linkCommand

Invoke-Checked -CommandLine @($objcopy, "-O", "ihex", $elfPath, $hexPath)
Invoke-Checked -CommandLine @($size, $elfPath)
Copy-Item -LiteralPath $hexPath -Destination $releaseHexPath -Force

Write-Host "Built firmware hex:" $releaseHexPath
