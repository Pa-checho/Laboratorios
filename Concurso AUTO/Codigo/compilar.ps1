param(
    # Directorio que contiene avr-gcc.exe, avr-objcopy.exe y avr-size.exe.
    [string]$AvrBin
)

$ErrorActionPreference = 'Stop'

if (-not $AvrBin) {
    $compiler = Get-Command avr-gcc.exe -ErrorAction SilentlyContinue
    if ($compiler) {
        $AvrBin = Split-Path -Parent $compiler.Source
    } else {
        $packages = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino\tools\avr-gcc'
        if (Test-Path -LiteralPath $packages) {
            $compiler = Get-ChildItem -LiteralPath $packages -Filter avr-gcc.exe -Recurse -File |
                Sort-Object FullName -Descending | Select-Object -First 1
            if ($compiler) { $AvrBin = $compiler.DirectoryName }
        }
        if (-not $AvrBin) {
            $legacy = Join-Path ${env:ProgramFiles(x86)} 'Arduino\hardware\tools\avr\bin'
            if (Test-Path -LiteralPath (Join-Path $legacy 'avr-gcc.exe')) {
                $AvrBin = $legacy
            }
        }
    }
}

if (-not $AvrBin) {
    throw 'No se encontro AVR-GCC. Indique su carpeta bin con -AvrBin.'
}

foreach ($tool in @('avr-gcc.exe', 'avr-objcopy.exe', 'avr-size.exe')) {
    if (-not (Test-Path -LiteralPath (Join-Path $AvrBin $tool))) {
        throw "Falta $tool en $AvrBin."
    }
}

$gcc = Join-Path $AvrBin 'avr-gcc.exe'
$objcopy = Join-Path $AvrBin 'avr-objcopy.exe'
$size = Join-Path $AvrBin 'avr-size.exe'
$source = Join-Path $PSScriptRoot 'main.c'
$build = Join-Path $PSScriptRoot 'build'
$elf = Join-Path $build 'auto_concurso.elf'
$hex = Join-Path $build 'auto_concurso.hex'

# Retirar solo salidas conocidas: una compilacion fallida no debe dejar un HEX antiguo.
foreach ($output in @($elf, $hex)) {
    if (Test-Path -LiteralPath $output) { Remove-Item -LiteralPath $output }
}

& $gcc --version
if ($LASTEXITCODE -ne 0) { throw 'No se pudo ejecutar AVR-GCC.' }
New-Item -ItemType Directory -Path $build -Force | Out-Null

& $gcc '-mmcu=atmega328p' '-DF_CPU=16000000UL' '-std=gnu11' '-Os' `
    '-Wall' '-Wextra' '-Werror' '-ffunction-sections' '-fdata-sections' `
    '-Wl,--gc-sections' $source '-o' $elf
if ($LASTEXITCODE -ne 0) { throw 'Fallo la compilacion. No se genero el HEX.' }

& $objcopy '-O' 'ihex' '-R' '.eeprom' $elf $hex
if ($LASTEXITCODE -ne 0) {
    if (Test-Path -LiteralPath $hex) { Remove-Item -LiteralPath $hex }
    throw 'Fallo la generacion del HEX.'
}

& $size '-C' '--mcu=atmega328p' $elf
if ($LASTEXITCODE -ne 0) { throw 'No se pudo informar el uso de memoria.' }
Write-Output "Compilacion terminada: $hex"
