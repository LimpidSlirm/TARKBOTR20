param([string]$PythonExecutable)
$ErrorActionPreference = 'Stop'
$visionPython = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'

if (!(Test-Path -LiteralPath $visionPython)) {
    $basePython = $PythonExecutable
    $baseArgs = @()
    if (!$basePython) {
        if (Get-Command py -ErrorAction SilentlyContinue) {
            $basePython = 'py'
            $baseArgs = @('-3')
        } elseif (Get-Command python -ErrorAction SilentlyContinue) {
            $basePython = 'python'
        } else {
            throw 'Install 64-bit Python 3.12, then run setup.ps1 again.'
        }
    }
    & $basePython @baseArgs -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 11) and sys.maxsize > 2**32 else 1)'
    if ($LASTEXITCODE -ne 0) {
        throw 'Python 3.11+ (64-bit) is required. Install Python 3.12 or use -PythonExecutable with its full path.'
    }
    & $basePython @baseArgs -m venv (Join-Path $PSScriptRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the Camera virtual environment.' }
}

# Also repair pip after an interrupted first installation.
& $visionPython -m ensurepip --upgrade
if ($LASTEXITCODE -ne 0) { throw 'Could not initialize pip in the Camera virtual environment.' }
& $visionPython -m pip install -r (Join-Path $PSScriptRoot 'requirements-tested.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check the network and pip output, then run setup.ps1 again.' }
& $visionPython -c 'import cv2, numpy; print(cv2.__version__, numpy.__version__)'
if ($LASTEXITCODE -ne 0) { throw 'Dependency import check failed.' }
Write-Host 'Camera is ready. Run run.ps1 --demo to preview generated lanes.'
