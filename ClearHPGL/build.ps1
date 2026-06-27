param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    # Use a Visual Studio solution generator by default (MSVC toolchain via VsDevCmd).
    # Pass "Ninja" for Ninja + single-config CMake layout (CMAKE_BUILD_TYPE at configure).
    [string]$Generator = "Visual Studio 17 2022",

    [string]$SourceDir = "",
    [string]$BuildDir = "",
    [string]$VcpkgToolchain = "",
    [string]$VsDevCmd = "",
    [string]$CmakeExe = "",
    [string]$WinDeployQtExe = "",

    [switch]$Clean,
    [switch]$SkipDeploy,
    [switch]$SkipConfigure
)

$ErrorActionPreference = "Stop"

function Resolve-PathOrThrow {
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

function Find-VsDevCmdBat {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "VsDevCmd")
    }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path -LiteralPath $vswhere) {
        $install = & $vswhere `
            -latest `
            -products * `
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
            -property installationPath 2>$null
        if (-not [string]::IsNullOrWhiteSpace($install)) {
            $bat = Join-Path $install "Common7\Tools\VsDevCmd.bat"
            if (Test-Path -LiteralPath $bat) {
                return $bat
            }
        }
    }
    foreach ($edition in @("Enterprise", "Professional", "Community", "Preview")) {
        $candidate = "C:/Program Files/Microsoft Visual Studio/2022/$edition/Common7/Tools/VsDevCmd.bat"
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'Could not locate VsDevCmd.bat. Install the "Desktop development with C++" workload or pass -VsDevCmd "...\VsDevCmd.bat".'
}

function Find-CMakeExe {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "CMake")
    }
    $qtCMake = "C:/Qt/Tools/CMake_64/bin/cmake.exe"
    if (Test-Path -LiteralPath $qtCMake) {
        return (Resolve-Path -LiteralPath $qtCMake).Path
    }
    $fromPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }
    throw "cmake.exe was not found. Install CMake, add it to PATH, or pass -CmakeExe."
}

function Find-VcpkgToolchain {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "vcpkg toolchain")
    }
    $defaultMachine = Join-Path ([Environment]::GetFolderPath([Environment+SpecialFolder]::UserProfile)) "vcpkg/scripts/buildsystems/vcpkg.cmake"
    if (Test-Path -LiteralPath $defaultMachine) {
        return (Resolve-Path -LiteralPath $defaultMachine).Path
    }
    throw 'vcpkg.cmake not found. Pass -VcpkgToolchain "...\vcpkg\scripts\buildsystems\vcpkg.cmake" or set up vcpkg in %USERPROFILE%\vcpkg.'
}

function Find-WinDeployQt {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "windeployqt")
    }
    $candidate = Join-Path ([Environment]::GetFolderPath([Environment+SpecialFolder]::UserProfile)) "vcpkg/installed/x64-windows/tools/Qt6/bin/windeployqt.exe"
    if (Test-Path -LiteralPath $candidate) {
        return (Resolve-Path -LiteralPath $candidate).Path
    }
    $fromPath = Get-Command windeployqt -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }
    throw "windeployqt.exe was not found. Pass -WinDeployQtExe or add Qt's bin folder to PATH."
}

function Resolve-ClearHPGLExe {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ResolvedBuildDir,
        [Parameter(Mandatory = $true)]
        [string]$BuildConfiguration
    )
    $configDir = Join-Path $ResolvedBuildDir $BuildConfiguration
    $roots = @(
        (Join-Path $ResolvedBuildDir "ClearHPGL.exe"),
        (Join-Path $configDir "ClearHPGL.exe")
    )
    foreach ($p in $roots) {
        if (Test-Path -LiteralPath $p) {
            return (Resolve-Path -LiteralPath $p).Path
        }
    }
    throw "Built executable ClearHPGL.exe not found under '$ResolvedBuildDir' (looked root and '$BuildConfiguration' subfolder)."
}

if ([string]::IsNullOrWhiteSpace($SourceDir)) {
    $SourceDir = $PSScriptRoot
}
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $PSScriptRoot "ProjectData/build"
}

$VsDevCmd = Find-VsDevCmdBat -UserPath $VsDevCmd
$CmakeExe = Find-CMakeExe -UserPath $CmakeExe
$VcpkgToolchain = Find-VcpkgToolchain -UserPath $VcpkgToolchain

$SourceDir = Resolve-PathOrThrow -PathValue $SourceDir -Name "SourceDir (ClearHPGL)"
if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host "Removing build directory: $BuildDir"
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}
if (-not (Test-Path -LiteralPath $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
}
$ResolvedBuildDir = (Resolve-Path -LiteralPath $BuildDir).Path

Write-Host "Building ClearHPGL..."
Write-Host "  Source: $SourceDir"
Write-Host "  Build : $ResolvedBuildDir"
Write-Host "  Config: $Config"
Write-Host "  Gen   : $Generator"

# VsDevCmd must be its own `call ... &&` segment; do not append cmake on the same line (parse_cmd rejects it).
$configureArgs = @(
    "`"$CmakeExe`"",
    "-S", "`"$SourceDir`"",
    "-B", "`"$ResolvedBuildDir`"",
    "-G", "`"$Generator`"",
    "-DCMAKE_TOOLCHAIN_FILE=`"$VcpkgToolchain`""
)
if ($Generator -match "Ninja") {
    $configureArgs += "-DCMAKE_BUILD_TYPE=$Config"
}

$buildArgs = @(
    "`"$CmakeExe`"",
    "--build", "`"$ResolvedBuildDir`"",
    "--target", "ClearHPGL",
    "--config", $Config
)

$cmdParts = New-Object System.Collections.Generic.List[string]
$cmdParts.Add("call `"$VsDevCmd`" -arch=x64 -host_arch=x64")
if (-not $SkipConfigure) {
    $cmdParts.Add(($configureArgs -join " "))
}
$cmdParts.Add(($buildArgs -join " "))
$cmd = ($cmdParts -join " && ")

Write-Host "Running inside VS dev shell (configure if enabled, then build)..."
cmd.exe /d /s /c $cmd
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE."
}

Write-Host "Build step completed successfully."

if (-not $SkipDeploy) {
    $WinDeployQtExe = Find-WinDeployQt -UserPath $WinDeployQtExe
    $appExe = Resolve-ClearHPGLExe -ResolvedBuildDir $ResolvedBuildDir -BuildConfiguration $Config
    Write-Host "Deploying Qt runtime dependencies to: $(Split-Path -Parent $appExe)"
    $deployMode = if ($Config -eq "Debug") { "--debug" } else { "--release" }
    & $WinDeployQtExe $deployMode --force --compiler-runtime $appExe
    if ($LASTEXITCODE -ne 0) {
        throw "windeployqt failed with exit code $LASTEXITCODE."
    }
    Write-Host "windeployqt completed."
} else {
    Write-Host "Skipping windeployqt (-SkipDeploy)."
}

Write-Host "Done."
