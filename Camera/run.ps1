param([Parameter(ValueFromRemainingArguments = $true)][string[]]$VisionArgs)
$ErrorActionPreference = 'Stop'
$visionPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
if (!(Test-Path -LiteralPath $visionPython)) {
    throw 'Camera environment is missing. Run: powershell -NoProfile -ExecutionPolicy Bypass -File "<Camera directory>/setup.ps1"'
}
& $visionPython (Join-Path $PSScriptRoot 'main.py') @VisionArgs
exit $LASTEXITCODE
