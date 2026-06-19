$ErrorActionPreference = "Stop"

$paths = @("build", "dist", "release")
foreach ($path in $paths) {
    if (Test-Path -LiteralPath $path) {
        Remove-Item -LiteralPath $path -Recurse -Force
    }
}

Get-ChildItem -LiteralPath . -Recurse -Directory -Filter "__pycache__" |
    Where-Object { $_.FullName -notmatch "\\.venv\\" } |
    Remove-Item -Recurse -Force
