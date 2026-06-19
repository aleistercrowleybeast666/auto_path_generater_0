param(
    [string]$DistDir = "dist\HJMB_Path_Generator_V4.0",
    [string]$Output = "release\HJMB_Path_Generator_V4.0_onedir.zip"
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path "."
$dist = Join-Path $root $DistDir
if (-not (Test-Path -LiteralPath $dist)) {
    throw "Dist directory not found: $dist"
}

$exe = Join-Path $dist "HJMB_Path_Generator.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    throw "Packaged executable not found: $exe"
}

$out = Join-Path $root $Output
$outDir = Split-Path -Parent $out
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

Compress-Archive -Path $dist -DestinationPath $out -Force
Get-FileHash -Algorithm SHA256 -LiteralPath $out
