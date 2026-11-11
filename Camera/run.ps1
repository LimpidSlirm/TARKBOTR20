param([Parameter(ValueFromRemainingArguments = $true)][string[]]$VisionArgs)
$ErrorActionPreference = 'Stop'
$visionPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
if (!(Test-Path -LiteralPath $visionPython)) {
    $visionPython = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
}
if (!(Test-Path -LiteralPath $visionPython)) {
    $visionPython = (Get-Command python -ErrorAction Stop).Source
}
& $visionPython (Join-Path $PSScriptRoot 'main.py') @VisionArgs
exit $LASTEXITCODE
