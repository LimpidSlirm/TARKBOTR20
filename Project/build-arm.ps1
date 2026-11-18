param([switch]$LinkOnly,[string]$KeilRoot=$env:KEIL_ROOT)
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$root = Split-Path $PSScriptRoot
$projectDir = Join-Path $root 'Project'
$output = Join-Path $projectDir 'Build'
New-Item -ItemType Directory -Path $output -Force | Out-Null
[xml]$project = Get-Content -LiteralPath (Join-Path $projectDir 'xproject.uvprojx')
$target = $project.Project.Targets.Target
$controls = $target.TargetOption.TargetArmAds.Cads.VariousControls
if (-not $KeilRoot) {
    if (Test-Path 'C:\Keil_v5\ARM\ARMCC\bin\armcc.exe') { $KeilRoot='C:\Keil_v5' }
    elseif (Test-Path 'D:\Keil_v5\ARM\ARMCC\bin\armcc.exe') { $KeilRoot='D:\Keil_v5' }
    else { throw 'ARM Compiler 5 not found; set KEIL_ROOT or pass -KeilRoot.' }
}
$bin = Join-Path $KeilRoot 'ARM\ARMCC\bin'
foreach ($tool in @('armcc.exe','armasm.exe','armlink.exe','fromelf.exe')) {
    if (-not (Test-Path -LiteralPath (Join-Path $bin $tool))) { throw ('Missing ARM tool: ' + $tool) }
}
$log = Join-Path $output 'compile.log'
if (-not $LinkOnly) { 'Competition firmware ARM Compiler 5.06 build; no hardware access.' | Set-Content $log }
$includes = @($controls.IncludePath.Split(';') | ForEach-Object { '-I' + [IO.Path]::GetFullPath((Join-Path $projectDir $_)) })
$defines = @($controls.Define.Split(',') | ForEach-Object { '-D' + $_ })
$objects = @()
$failures = 0
foreach ($file in $target.Groups.Group.Files.File) {
    $source = [IO.Path]::GetFullPath((Join-Path $projectDir $file.FilePath))
    $ext = [IO.Path]::GetExtension($source)
    if ($ext -notin @('.c','.s')) { continue }
    $obj = Join-Path $output ([IO.Path]::GetFileNameWithoutExtension($source)+'.o')
    $objects += $obj
    if ($LinkOnly) { if(-not(Test-Path $obj)) { throw ('Missing object: '+$obj) }; continue }
    # Windows PowerShell 5 wraps native stderr warnings as ErrorRecord objects.
    # Keep checking native exit codes without treating a warning as termination.
    $ErrorActionPreference = 'Continue'
    if ($ext -eq '.c') {
        $arguments = @('--c99','--cpu=Cortex-M4','--fpu=FPv4-SP','--apcs=interwork','--split_sections','-O0','-g','-D__MICROLIB') + $defines + $includes + @('-c',$source,'-o',$obj)
        $messages = & (Join-Path $bin 'armcc.exe') @arguments 2>&1
    } else {
        $arguments = @('--cpu=Cortex-M4','--fpu=FPv4-SP','-g','--pd','__MICROLIB SETA 1','-o',$obj,$source)
        $messages = & (Join-Path $bin 'armasm.exe') @arguments 2>&1
    }
    $rc = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    ('SOURCE: ' + $source) | Add-Content $log
    $messages | Add-Content $log
    if ($rc -ne 0) { $failures++; Write-Output ('FAIL: ' + $file.FileName); $messages | Write-Output }
}
Write-Output ('Compiled units: '+$objects.Count+'; failed units: '+$failures)
if ($failures -ne 0) { exit 1 }
$scatter = Join-Path $output 'competition.sct'
@'
LR_IROM1 0x08000000 0x00080000 {
  ER_IROM1 0x08000000 0x00080000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000 {
    .ANY (+RW +ZI)
  }
}
'@ | Set-Content $scatter -Encoding ascii
$response = Join-Path $output 'objects.rsp'
$objects | ForEach-Object { '"'+[IO.Path]::GetFileName($_)+'"' } | Set-Content $response -Encoding ascii
$image = Join-Path $output 'competition.axf'
Push-Location $output
$ErrorActionPreference = 'Continue'
$messages = & (Join-Path $bin 'armlink.exe') --cpu=Cortex-M4 --library_type=microlib --strict --entry=Reset_Handler --scatter competition.sct --map --symbols --info=sizes,totals,unused,veneers --list competition.map --via objects.rsp -o competition.axf 2>&1
$rc = $LASTEXITCODE
$ErrorActionPreference = 'Stop'
Pop-Location
$messages | Tee-Object -FilePath (Join-Path $output 'link.log')
if ($rc -ne 0) { exit $rc }
$ErrorActionPreference = 'Continue'
& (Join-Path $bin 'fromelf.exe') --i32combined --output (Join-Path $output 'competition.hex') $image
$rc = $LASTEXITCODE
$ErrorActionPreference = 'Stop'
if ($rc -ne 0) { exit $rc }
Select-String -LiteralPath $log -Pattern 'warning:|error:' | ForEach-Object { $_.Line }
Write-Output ('ARM image generated: ' + $image)

