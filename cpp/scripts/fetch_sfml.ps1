param(
    [ValidateSet("auto", "mingw", "msvc")]
    [string]$Toolchain = "auto"
)

# Downloads SFML 2.6.1 prebuilt libraries into cpp/third_party/sfml/
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1
#   powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1 -Toolchain mingw
#   powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1 -Toolchain msvc

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Dest = Join-Path $Root "third_party\sfml"

if ($Toolchain -eq "auto") {
    if (Get-Command g++ -ErrorAction SilentlyContinue) {
        $Toolchain = "mingw"
    } elseif (Get-Command cl -ErrorAction SilentlyContinue) {
        $Toolchain = "msvc"
    } else {
        throw "Could not detect g++ or cl. Pass -Toolchain mingw or -Toolchain msvc explicitly."
    }
}

if ($Toolchain -eq "mingw") {
    $ArchiveName = "SFML-2.6.1-windows-gcc-13.1.0-mingw-64-bit.zip"
    Write-Host "Downloading SFML 2.6.1 (MinGW GCC 13.1 64-bit)..."
} else {
    $ArchiveName = "SFML-2.6.1-windows-vc17-64-bit.zip"
    Write-Host "Downloading SFML 2.6.1 (MSVC vc17 64-bit)..."
}

$ZipUrl = "https://github.com/SFML/SFML/releases/download/2.6.1/$ArchiveName"
$TempZip = Join-Path $env:TEMP $ArchiveName
$TempExtract = Join-Path $env:TEMP "sfml-extract"

Invoke-WebRequest -Uri $ZipUrl -OutFile $TempZip -UseBasicParsing

if (Test-Path $Dest) {
    Remove-Item -Recurse -Force $Dest
}
New-Item -ItemType Directory -Path $Dest -Force | Out-Null

if (Test-Path $TempExtract) {
    Remove-Item -Recurse -Force $TempExtract
}
Expand-Archive -Path $TempZip -DestinationPath $TempExtract -Force

$Inner = Get-ChildItem -Path $TempExtract -Directory | Select-Object -First 1
Copy-Item -Path (Join-Path $Inner.FullName "*") -Destination $Dest -Recurse -Force

Remove-Item $TempZip -Force -ErrorAction SilentlyContinue
Remove-Item $TempExtract -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "SFML installed to: $Dest"
Write-Host "Contents:"
Get-ChildItem $Dest | Format-Table Name
