$ErrorActionPreference = "Stop"

$workspace = Split-Path -Parent $PSScriptRoot
$output = Join-Path $env:TEMP "chassis_core_tests.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I (Join-Path $workspace "common") `
    -I (Join-Path $workspace "firmware\stm32f407\inc") `
    (Join-Path $workspace "common\chassis_can_protocol.c") `
    (Join-Path $workspace "firmware\stm32f407\src\chassis_params.c") `
    (Join-Path $workspace "firmware\stm32f407\src\wheel_controller.c") `
    (Join-Path $workspace "firmware\stm32f407\src\chassis_app.c") `
    (Join-Path $workspace "tests\test_main.c") `
    -lm `
    -o $output

if ($LASTEXITCODE -ne 0) {
    throw "C source compilation failed."
}

& $output
if ($LASTEXITCODE -ne 0) {
    throw "C unit tests failed."
}

# Check C99 syntax and types in the C30D/FreeRTOS adapter with stub headers.
& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -fsyntax-only `
    -I (Join-Path $workspace "tests\stubs") `
    -I (Join-Path $workspace "common") `
    -I (Join-Path $workspace "firmware\stm32f407\inc") `
    (Join-Path $workspace "firmware\stm32f407\src\c30d_chassis_port.c") `
    (Join-Path $workspace "firmware\stm32f407\src\chassis_tasks.c")

if ($LASTEXITCODE -ne 0) {
    throw "C30D adapter syntax check failed."
}
