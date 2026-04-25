param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    [string]$Generator = "Ninja",

    [string]$SourceDir = "",
    [string]$BuildDir = "",
    [string]$VcpkgToolchain = "",
    [string]$VsDevCmd = "",
    [string]$CmakeExe = "",
    [string]$WinDeployQtExe = "",

    [switch]$Clean
)

$ErrorActionPreference = "Stop"

function Resolve-RequiredPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PathValue,
        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    $resolved = Resolve-Path -LiteralPath $PathValue -ErrorAction SilentlyContinue
    if (-not $resolved) {
        throw "$Name was not found: $PathValue"
    }
    return $resolved.Path
}

if ([string]::IsNullOrWhiteSpace($SourceDir)) {
    $SourceDir = Join-Path $PSScriptRoot "ClearCNC_Controller/QtController"
}
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $PSScriptRoot ("ClearCNC_Controller/" + $Config)
}
if ([string]::IsNullOrWhiteSpace($VcpkgToolchain)) {
    $VcpkgToolchain = "C:/Users/Adam/vcpkg/scripts/buildsystems/vcpkg.cmake"
}
if ([string]::IsNullOrWhiteSpace($VsDevCmd)) {
    $VsDevCmd = "C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/Tools/VsDevCmd.bat"
}
if ([string]::IsNullOrWhiteSpace($CmakeExe)) {
    $CmakeExe = "C:/Qt/Tools/CMake_64/bin/cmake.exe"
}
if ([string]::IsNullOrWhiteSpace($WinDeployQtExe)) {
    $WinDeployQtExe = "C:/Users/Adam/vcpkg/installed/x64-windows/tools/Qt6/bin/windeployqt.exe"
}

$SourceDir = Resolve-RequiredPath -PathValue $SourceDir -Name "SourceDir"
$VcpkgToolchain = Resolve-RequiredPath -PathValue $VcpkgToolchain -Name "VcpkgToolchain"
$VsDevCmd = Resolve-RequiredPath -PathValue $VsDevCmd -Name "VsDevCmd"
$CmakeExe = Resolve-RequiredPath -PathValue $CmakeExe -Name "CmakeExe"
$WinDeployQtExe = Resolve-RequiredPath -PathValue $WinDeployQtExe -Name "WinDeployQtExe"

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host "Removing build directory: $BuildDir"
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}
if (-not (Test-Path -LiteralPath $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
}

Write-Host "Building ClearCNC Qt controller..."
Write-Host "  Source: $SourceDir"
Write-Host "  Build : $BuildDir"
Write-Host "  Config: $Config"
Write-Host "  Gen   : $Generator"

$configureArgs = @(
    "`"$CmakeExe`"",
    "-S", "`"$SourceDir`"",
    "-B", "`"$BuildDir`"",
    "-G", "`"$Generator`"",
    "-DCMAKE_TOOLCHAIN_FILE=`"$VcpkgToolchain`""
)
if ($Generator -eq "Ninja") {
    $configureArgs += "-DCMAKE_BUILD_TYPE=$Config"
}

$buildArgs = @(
    "`"$CmakeExe`"",
    "--build", "`"$BuildDir`"",
    "--target", "ClearCNC_Controller",
    "--config", $Config
)

$cmd = @(
    "call `"$VsDevCmd`" -arch=x64 -host_arch=x64",
    ($configureArgs -join " "),
    ($buildArgs -join " ")
) -join " && "

Write-Host "Running configure + build inside VS dev shell..."
cmd.exe /d /s /c $cmd
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE."
}

Write-Host "Build step completed successfully."
Write-Host "Deploying Qt runtime dependencies..."

$appExeRoot = Join-Path $BuildDir "ClearCNC_Controller.exe"
$appExeConfig = Join-Path (Join-Path $BuildDir $Config) "ClearCNC_Controller.exe"
if (Test-Path -LiteralPath $appExeRoot) {
    $appExe = $appExeRoot
} elseif (Test-Path -LiteralPath $appExeConfig) {
    $appExe = $appExeConfig
} else {
    throw "Built executable not found in '$BuildDir'."
}

$deployMode = if ($Config -eq "Debug") { "--debug" } else { "--release" }
& $WinDeployQtExe $deployMode --force --compiler-runtime $appExe
if ($LASTEXITCODE -ne 0) {
    throw "windeployqt failed with exit code $LASTEXITCODE."
}

Write-Host "Build completed successfully."
