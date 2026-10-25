# Builds, tests and packages TrailForge for Windows x64 as a zip (trailforge.exe + assets).
#
#   pwsh tools/release/package_windows.ps1                # build + test + package into dist/
#   pwsh tools/release/package_windows.ps1 -SmokeWindow   # additionally start the window for a few seconds
#                                                         # (needs a D3D12 or Vulkan capable GPU)
#
# Run from an "x64 Native Tools" developer prompt (MSVC cl, Ninja and CMake on PATH; CI uses
# ilammy/msvc-dev-cmd). Steps: clean Release configure in build/package with TRAILFORGE_PACKAGED=ON (assets are read
# only from next to the executable), build, ctest, assemble dist/TrailForge-<version>-windows-x64/ (trailforge.exe,
# assets/{audio,music,fonts,images,vehicles}, README.txt), check that the executable has no source-tree references, zip it, then smoke test a
# fresh extraction from another working directory (--version, --paths: the asset directory must be the one inside
# the package). The version comes from project(TrailForge VERSION x.y.z) in CMakeLists.txt.
param([switch]$SmokeWindow)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$Build = Join-Path $Root 'build\package'
$Dist = Join-Path $Root 'dist'

function Step($msg) { Write-Host "`n=== $msg" }
function Fail($msg) { Write-Error "FAIL: $msg"; exit 1 }
# Simple function (no param block) so that native options such as -S/-B/-G reach the command untouched.
function Run {
    $exe = $args[0]
    $rest = @($args | Select-Object -Skip 1)
    & $exe @rest
    if ($LASTEXITCODE -ne 0) { Fail "command failed ($LASTEXITCODE): $($args -join ' ')" }
}

$m = Select-String -Path (Join-Path $Root 'CMakeLists.txt') -Pattern '^project\(TrailForge VERSION ([0-9.]+)'
if (-not $m) { Fail 'cannot read the version from CMakeLists.txt' }
$Version = $m.Matches[0].Groups[1].Value
$Name = "TrailForge-$Version-windows-x64"
$Pkg = Join-Path $Dist $Name
$Zip = Join-Path $Dist "$Name.zip"

Step "TrailForge $Version (x64): clean Release configure"
if (Test-Path $Build) { Remove-Item -Recurse -Force $Build }
Run cmake -S $Root -B $Build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=cl -DCMAKE_CXX_COMPILER=cl `
    -DTRAILFORGE_PACKAGED=ON -DTRAILFORGE_BUILD_APP=ON -DTRAILFORGE_BUILD_TESTS=ON

Step 'build'
Run cmake --build $Build

Step 'ctest'
Run ctest --test-dir $Build --output-on-failure

Step "assemble dist/$Name"
$Exe = Join-Path $Build 'trailforge.exe'
$ver = & $Exe --version
if ($ver -ne "TrailForge $Version") { Fail "binary version mismatch: '$ver'" }
New-Item -ItemType Directory -Force $Dist | Out-Null
if (Test-Path $Pkg) { Remove-Item -Recurse -Force $Pkg }
if (Test-Path $Zip) { Remove-Item -Force $Zip }
New-Item -ItemType Directory -Force (Join-Path $Pkg 'assets') | Out-Null
Copy-Item $Exe $Pkg
foreach ($sub in 'audio', 'music', 'fonts', 'images', 'vehicles') {
    Copy-Item -Recurse (Join-Path $Root "assets\$sub") (Join-Path $Pkg "assets\$sub")
}
# The logo is copied from external_assets/images into the build tree at configure time.
Copy-Item (Join-Path $Build 'assets\images\logo.png') (Join-Path $Pkg 'assets\images\logo.png')
@"
TrailForge $Version for Windows (x64)

UNTESTED ON REAL HARDWARE: this build was compiled and its headless tests passed in CI,
but it has not been run on a real Windows machine with a GPU.

Run:  trailforge.exe            (needs a Direct3D 12 or Vulkan capable GPU and driver)
      trailforge.exe --help     (options), trailforge.exe --paths (asset and user data directories)
Keep the assets folder next to the executable. The executable is not code-signed, so Windows SmartScreen
may warn on first start ("More info" -> "Run anyway").
"@ | Set-Content -Encoding ascii (Join-Path $Pkg 'README.txt')
foreach ($f in 'audio\manifest.json', 'music\intro.mp3', 'fonts\Rajdhani-SemiBold.ttf', 'fonts\OFL.txt', 'images\logo.png',
            'vehicles\trailforge_suv\vehicle.json') {
    if (-not (Test-Path (Join-Path $Pkg "assets\$f"))) { Fail "assets\$f missing in the package" }
}
if (Select-String -Path (Join-Path $Pkg 'trailforge.exe') -SimpleMatch -Quiet -Pattern (Join-Path $Root 'assets')) {
    Fail 'the packaged executable still references the source tree assets'
}

Step 'zip'
Compress-Archive -Path $Pkg -DestinationPath $Zip
Get-Item $Zip | Format-Table Name, Length
(Get-FileHash -Algorithm SHA256 $Zip).Hash

Step 'smoke test: fresh extraction, run from another directory'
$Extract = Join-Path ([System.IO.Path]::GetTempPath()) ("trailforge_unzip_" + [guid]::NewGuid())
Expand-Archive -Path $Zip -DestinationPath $Extract
$Bin = Join-Path $Extract "$Name\trailforge.exe"
Push-Location ([System.IO.Path]::GetTempPath())
try {
    Run $Bin --version
    $paths = & $Bin --paths
    if ($LASTEXITCODE -ne 0) { Fail '--paths failed' }
    $paths | ForEach-Object { Write-Host $_ }
    $assetLines = @($paths | Where-Object { $_ -like 'assets: *' })
    if ($assetLines.Count -ne 1) { Fail 'packaged build must have exactly one asset directory' }
    $assetDir = $assetLines[0].Substring('assets: '.Length)
    if (-not (Test-Path (Join-Path $assetDir 'manifest.json'))) { Fail "asset directory $assetDir has no manifest.json" }
    $musicDir = @($paths | Where-Object { $_ -like 'music: *' })[0].Substring('music: '.Length)
    if (-not (Test-Path (Join-Path $musicDir 'intro.mp3'))) { Fail "music directory $musicDir has no intro.mp3" }
    if ($SmokeWindow) {
        Step 'smoke test: window'
        $log = Join-Path ([System.IO.Path]::GetTempPath()) 'trailforge_smoke.log'
        & $Bin --no-ui --quit-after 4 --audio-capture "$log.wav" *> $log
        $code = $LASTEXITCODE
        Get-Content $log
        if ($code -ne 0) { Fail 'window smoke run failed' }
    }
} finally {
    Pop-Location
}
Remove-Item -Recurse -Force $Extract

Step "done: $Zip"
