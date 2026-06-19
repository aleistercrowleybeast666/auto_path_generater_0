param(
    [string]$Spec = "packaging\HJMB_Path_Generator.spec"
)

$ErrorActionPreference = "Stop"
python -m PyInstaller --noconfirm $Spec
