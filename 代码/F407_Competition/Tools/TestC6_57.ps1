param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$sourceNames = @('app_console','arm_bench','arm_units','competition','emm_bus','mission',
                 'motor_bench','qr_bridge','route_library','route_scene','route2','servo')
$sources = $sourceNames | ForEach-Object { "Core/Src/$_.c" }
# Relative compiler paths also work when the repository path contains Chinese.
Push-Location $projectRoot
try {
    & $Compiler -std=c11 -O0 -ITests/host/include -ICore/Inc `
        Tests/host/test_C6_57.c Tests/host/servo_hal_stub.c @sources -o Tests/host/test_C6_57.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & './Tests/host/test_C6_57.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Host verification failed.' }
} finally { Pop-Location }
