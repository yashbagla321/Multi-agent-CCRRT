# Downloads SFML 2.6.1 prebuilt libraries into cpp/third_party/sfml/
# Usage: powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Dest = Join-Path $Root "third_party\sfml"
$ZipUrl = "https://github.com/SFML/SFML/releases/download/2.6.1/SFML-2.6.1-windows-vc17-64-bit.zip"
$TempZip = Join-Path $env:TEMP "SFML-2.6.1-windows-vc17-64-bit.zip"
$TempExtract = Join-Path $env:TEMP "sfml-extract"

Write-Host "Downloading SFML 2.6.1 (MSVC 64-bit)..."
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
