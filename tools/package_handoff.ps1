param([string]$DestinationName = 'C30D_Development_Handoff_20260909')
$ErrorActionPreference = 'Stop'
$sourceRoot = Split-Path $PSScriptRoot -Parent
$targetRoot = Join-Path (Join-Path $sourceRoot 'releases') $DestinationName
if (Test-Path -LiteralPath $targetRoot) { throw "Destination already exists: $targetRoot" }
New-Item -ItemType Directory -Path $targetRoot | Out-Null
$skipDirectory = '(^|[\\/])(OBJ[^\\/]*|__pycache__|\.pytest_cache|\.git|build|install|log)([\\/]|$)'
$skipFile = '\.(o|obj|d|crf|dep|pyc|pyo|axf|lnp|map|bak)$|\.uvguix\.|(^|[\\/])Thumbs\.db$'
foreach ($folder in @('common','firmware','docs','keil_project','ros2_ws','tests','tools','hardware_backups')) {
    foreach ($file in Get-ChildItem -LiteralPath (Join-Path $sourceRoot $folder) -Recurse -File -Force) {
        $relative = $file.FullName.Substring($sourceRoot.Length + 1)
        if ($relative -match $skipDirectory -or $relative -match $skipFile) { continue }
        $target = Join-Path $targetRoot $relative
        New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $target
        if ((Get-FileHash -LiteralPath $file.FullName).Hash -ne (Get-FileHash -LiteralPath $target).Hash) { throw "Copy mismatch: $relative" }
    }
}
foreach ($name in @('README.md','LICENSE','CMakeLists.txt','.gitignore')) {
    Copy-Item -LiteralPath (Join-Path $sourceRoot $name) -Destination (Join-Path $targetRoot $name)
}
# Supporting manuals are copied without modifying their original content.
$vendorRoot = 'C:\Users\User\Desktop\ROS机器人小车资料\2.WHEELTEC R550-V550 ROS教育机器人运动底盘资料'
if (Test-Path -LiteralPath $vendorRoot) {
    foreach ($file in Get-ChildItem -LiteralPath $vendorRoot -Recurse -File) {
        $relative = $file.FullName.Substring($vendorRoot.Length + 1)
        $include = ($relative -match '^4\.芯片数据手册与原理图[\\/]' -and $file.Extension -match '^\.(pdf|png|jpg|txt)$') -or
                   ($relative -notmatch '[\\/]' -and $file.Extension -eq '.pdf') -or
                   ($file.Name -eq '1.Mini小车接线说明.pdf')
        if (-not $include) { continue }
        $target = Join-Path $targetRoot (Join-Path 'reference_materials\WHEELTEC' $relative)
        New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $target
        if ((Get-FileHash -LiteralPath $file.FullName).Hash -ne (Get-FileHash -LiteralPath $target).Hash) { throw "Reference mismatch: $relative" }
    }
}
# Historical binaries are for comparison, not an automatically selected firmware.
$historical = Join-Path $targetRoot 'historical_builds'
foreach ($project in @('R550_C30D_SERIAL_MOTOR','R550_C30D_CAN_MOTOR')) {
    $projectRoot = Join-Path $sourceRoot "keil_project\$project"
    foreach ($file in Get-ChildItem -LiteralPath $projectRoot -Recurse -File | Where-Object { $_.Directory.Name -match '^OBJ' -and ($_.Extension -eq '.hex' -or $_.Name -eq 'keil_build.log') }) {
        $relative = $file.FullName.Substring($projectRoot.Length + 1)
        $target = Join-Path $historical (Join-Path $project $relative)
        New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $target
    }
}
Write-Output $targetRoot
