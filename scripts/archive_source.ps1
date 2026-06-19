param(
    [string]$Output = "release\HJMB_Path_Generator_V4.0_source.zip"
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path "."
$out = Join-Path $root $Output
$outDir = Split-Path -Parent $out
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
if (Test-Path -LiteralPath $out) {
    Remove-Item -LiteralPath $out -Force
}

$excludeParts = @("\.git\", "\.venv\", "\__pycache__\", "\build\", "\dist\", "\release\")
$files = Get-ChildItem -LiteralPath $root -Recurse -File | Where-Object {
    $full = $_.FullName
    -not ($excludeParts | Where-Object { $full.Contains($_) })
}

Compress-Archive -Path $files.FullName -DestinationPath $out -Force
Get-FileHash -Algorithm SHA256 -LiteralPath $out
