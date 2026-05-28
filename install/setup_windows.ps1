# TeLeRobot Windows + WSL setup bootstrap
# Run from the project root with:
#   powershell -ExecutionPolicy Bypass -File install\setup_windows.ps1

$ErrorActionPreference = "Stop"

function Write-Step($Message) {
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Write-Ok($Message) {
    Write-Host "[OK] $Message" -ForegroundColor Green
}

function Write-Skip($Message) {
    Write-Host "[SKIP] $Message" -ForegroundColor Yellow
}

function Write-Warn($Message) {
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Require-Command($Command, $InstallHint) {
    if (-not (Get-Command $Command -ErrorAction SilentlyContinue)) {
        Write-Host "[MISSING] $Command" -ForegroundColor Red
        Write-Host $InstallHint -ForegroundColor Yellow
        throw "Required command missing: $Command"
    }

    Write-Ok "$Command found"
}

function Convert-ToWslPath($WindowsPath) {
    $full = [System.IO.Path]::GetFullPath($WindowsPath)
    $drive = $full.Substring(0, 1).ToLower()
    $rest = $full.Substring(2).Replace([char]92, '/')
    return "/mnt/$drive$rest"
}

function Refresh-Path {
    $machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machinePath;$userPath"
}


function Ensure-WslCommandAvailable {
    Write-Step "Checking WSL availability"

    if (Get-Command wsl -ErrorAction SilentlyContinue) {
        Write-Ok "wsl found"
        return
    }

    Write-Host "[MISSING] wsl" -ForegroundColor Red
    Write-Host ""
    Write-Warn "Windows Subsystem for Linux is not installed or not available in PATH."
    Write-Host "Please install WSL manually, then rerun this setup:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  Open PowerShell as Administrator and run:" -ForegroundColor Green
    Write-Host "    wsl --install" -ForegroundColor Green
    Write-Host ""
    Write-Host "After installation/reboot, rerun Setup_TeLeRobot.bat." -ForegroundColor Yellow
    throw "WSL is missing. Please install WSL manually and rerun setup."
}

function Test-PythonCommand {
    param([string]$Command)

    try {
        $result = cmd /c "$Command --version" 2>&1
        $text = ($result | Out-String)

        if ($LASTEXITCODE -eq 0 -and $text -match "Python\s+3\.") {
            return $true
        }

        return $false
    } catch {
        return $false
    }
}

function Get-WorkingWindowsPythonCommand {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        if (Test-PythonCommand "py -3") {
            return "py -3"
        }
    }

    if (Get-Command python -ErrorAction SilentlyContinue) {
        if (Test-PythonCommand "python") {
            return "python"
        }
    }

    return $null
}

function Ensure-WindowsPython {
    Write-Step "Checking Windows Python"

    $workingPython = Get-WorkingWindowsPythonCommand

    if ($workingPython) {
        Write-Ok "Windows Python found: $workingPython"
        $script:PythonCmd = $workingPython
        return
    }

    Write-Warn "A working Windows Python was not found."
    Write-Host "Attempting to install Python 3.12 using winget..." -ForegroundColor Cyan

    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        Write-Host "[MISSING] winget" -ForegroundColor Red
        Write-Host "Install Python 3.12 manually from https://www.python.org/downloads/windows/ and rerun setup." -ForegroundColor Yellow
        throw "Windows Python missing and winget unavailable."
    }

    winget install -e --id Python.Python.3.12

    if ($LASTEXITCODE -ne 0) {
        throw "Python 3.12 installation failed. Install Python manually and rerun setup."
    }

    Write-Host "Refreshing PowerShell PATH..." -ForegroundColor Cyan
    Refresh-Path

    $workingPython = Get-WorkingWindowsPythonCommand

    if ($workingPython) {
        Write-Ok "Windows Python found after install: $workingPython"
        $script:PythonCmd = $workingPython
        return
    }

    Write-Warn "Python was installed, but this PowerShell session cannot see it yet."
    throw "Close this PowerShell window, reopen it, and rerun Setup_TeLeRobot.bat."
}

function Get-WslDistros {
    if (-not (Get-Command wsl -ErrorAction SilentlyContinue)) {
        return @()
    }

    return @(
        wsl --list --quiet 2>$null |
        ForEach-Object { ($_ -replace "`0", "").Trim() } |
        Where-Object { $_ }
    )
}

function Get-TelerobotWslDistro {
    $distros = Get-WslDistros

    foreach ($candidate in @("Ubuntu-24.04", "Ubuntu-22.04", "Ubuntu")) {
        if ($distros -contains $candidate) {
            $rawVersion = ""

            try {
                # Avoid nested bash/python quoting. Call python3 directly.
                $rawVersion = (wsl -d $candidate -- python3 --version 2>$null)
            } catch {
                Write-Warn "Could not query Python version for WSL distro '$candidate'."
                continue
            }

            $version = "unknown"

            if ($rawVersion -match "Python\s+(\d+\.\d+)") {
                $version = $Matches[1]
            }

            if ($version -eq "3.12") {
                return $candidate
            }

            Write-Warn "Found WSL distro '$candidate' but it uses Python $version. TeLeRobot release lock expects Python 3.12."
        }
    }

    return $null
}

function Install-UbuntuIfMissing {
    $ubuntu = Get-TelerobotWslDistro

    if ($ubuntu) {
        Write-Ok "Compatible Ubuntu WSL distro found: $ubuntu"
        return $ubuntu
    }

    Write-Warn "No compatible Ubuntu WSL distro found."
    Write-Host "TeLeRobot needs Ubuntu 24.04 with Python 3.12 for the pinned dependency lock." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Attempting to install Ubuntu 24.04 using: wsl --install -d Ubuntu-24.04" -ForegroundColor Cyan
    Write-Host ""
    Write-Warn "If Ubuntu opens and asks for a Unix username/password, create them."
    Write-Warn "When you reach the Ubuntu prompt, type: exit"
    Write-Host ""

    wsl --install -d Ubuntu-24.04 | Out-Host

    if ($LASTEXITCODE -ne 0) {
        throw "Ubuntu 24.04 installation failed. Install manually with: wsl --install -d Ubuntu-24.04"
    }

    Start-Sleep -Seconds 2

    $ubuntu = Get-TelerobotWslDistro

    if (-not $ubuntu) {
        throw "Ubuntu 24.04 was installed but is not ready. Open Ubuntu 24.04 once, create the Linux username/password, type exit, then rerun setup."
    }

    wsl -d $ubuntu -- bash -lc "echo ubuntu-ready" | Out-Host

    if ($LASTEXITCODE -ne 0) {
        throw "Ubuntu exists but is not initialized. Open it once, create username/password, type exit, then rerun setup."
    }

    Write-Ok "Ubuntu WSL distro is ready: $ubuntu"
    return $ubuntu
}


function Ensure-WslMirroredNetworking {
    Write-Step "Ensuring WSL mirrored networking"

    $wslConfig = Join-Path $env:USERPROFILE ".wslconfig"
    $needsWrite = $true

    if (Test-Path $wslConfig) {
        $existing = Get-Content $wslConfig -Raw -ErrorAction SilentlyContinue
        if ($existing -match "networkingMode\s*=\s*mirrored") {
            Write-Ok "WSL mirrored networking already enabled"
            $needsWrite = $false
        }
    }

    if ($needsWrite) {
        $existing = ""
        if (Test-Path $wslConfig) {
            $existing = Get-Content $wslConfig -Raw -ErrorAction SilentlyContinue
        }

        if ($existing -match "\[wsl2\]") {
            if ($existing -match "networkingMode\s*=") {
                $existing = $existing -replace "networkingMode\s*=.*", "networkingMode=mirrored"
            } else {
                $existing = $existing -replace "\[wsl2\]", "[wsl2]`r`nnetworkingMode=mirrored"
            }
            Set-Content -Path $wslConfig -Value $existing -Encoding ASCII
        } else {
@"
[wsl2]
networkingMode=mirrored
"@ | Set-Content -Path $wslConfig -Encoding ASCII
        }

        Write-Ok "Enabled WSL mirrored networking in $wslConfig"
        Write-Warn "Restarting WSL to apply mirrored networking..."
        wsl --shutdown
    }
}

function Ensure-FirewallPort {
    param(
        [int]$Port = 8765,
        [string]$RuleName = "TeLeRobot 8765"
    )

    Write-Step "Ensuring Windows firewall allows TeLeRobot port $Port"

    $rule = Get-NetFirewallRule -DisplayName $RuleName -ErrorAction SilentlyContinue

    if ($rule) {
        Write-Ok "Firewall rule already exists: $RuleName"
        return
    }

    try {
        New-NetFirewallRule -DisplayName $RuleName -Direction Inbound -Protocol TCP -LocalPort $Port -Action Allow | Out-Null
        Write-Ok "Firewall rule added: $RuleName"
    } catch {
        Write-Warn "Could not add firewall rule without elevation. Requesting administrator permission..."

        $cmd = "New-NetFirewallRule -DisplayName '$RuleName' -Direction Inbound -Protocol TCP -LocalPort $Port -Action Allow"
        Start-Process PowerShell -Verb RunAs -Wait -ArgumentList "-NoProfile -ExecutionPolicy Bypass -Command `"$cmd`""

        $rule = Get-NetFirewallRule -DisplayName $RuleName -ErrorAction SilentlyContinue
        if ($rule) {
            Write-Ok "Firewall rule added after elevation: $RuleName"
        } else {
            throw "Failed to create firewall rule for port $Port. Add it manually or run setup as Administrator."
        }
    }
}

function Invoke-WslScript($ScriptText) {
    if (-not $script:WslDistro) {
        throw "No WSL distro selected."
    }

    $tempFile = Join-Path $env:TEMP ("telerobot_setup_" + [System.Guid]::NewGuid().ToString() + ".sh")

    try {
        # Force LF line endings. Bash does not like PowerShell CRLF here-strings.
        $cleanScript = $ScriptText -replace "`r`n", "`n"
        $cleanScript = $cleanScript -replace "`r", "`n"

        $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
        [System.IO.File]::WriteAllText($tempFile, $cleanScript, $utf8NoBom)

        $tempFileWsl = Convert-ToWslPath $tempFile

        wsl -d $script:WslDistro -- bash $tempFileWsl

        if ($LASTEXITCODE -ne 0) {
            throw "WSL script failed."
        }
    }
    finally {
        if (Test-Path $tempFile) {
            Remove-Item -Force $tempFile
        }
    }
}

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$ProjectRootWin = $ProjectRoot.Path
$ProjectRootWsl = Convert-ToWslPath $ProjectRootWin

Write-Host "TeLeRobot Windows + WSL setup" -ForegroundColor Cyan
Write-Host "Project root: $ProjectRootWin"
Write-Host "WSL path:     $ProjectRootWsl"

Write-Step "Checking Windows prerequisites"

Ensure-WslCommandAvailable

if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Write-Warn "usbipd-win is missing."
    Write-Host "usbipd-win is required for attaching the robot arm and cameras to WSL." -ForegroundColor Yellow

    if (Get-Command winget -ErrorAction SilentlyContinue) {
        Write-Host "Attempting to install usbipd-win using winget..." -ForegroundColor Cyan
        Write-Host "A Windows installer window or UAC prompt may appear." -ForegroundColor Yellow

        winget install --interactive --exact dorssel.usbipd-win

        if ($LASTEXITCODE -ne 0) {
            throw "usbipd-win installation failed. Install it manually from https://github.com/dorssel/usbipd-win/releases and rerun setup."
        }

        Write-Host ""
        Write-Warn "usbipd-win was installed or updated."
        Write-Host "Refreshing PowerShell PATH..." -ForegroundColor Cyan
        Refresh-Path
    } else {
        throw "usbipd-win is missing and winget was not found. Install usbipd-win manually and rerun setup."
    }
}

if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    throw "usbipd was installed but is not available in this PowerShell session. Open a new PowerShell window and rerun setup_windows.ps1."
}

Write-Ok "usbipd found"

Ensure-WindowsPython

Write-Step "Checking/registering Ubuntu WSL distro"

$script:WslDistro = Install-UbuntuIfMissing
Write-Ok "Using WSL distro: $script:WslDistro"

Write-Host "Setting Ubuntu as default WSL distro for launcher compatibility..." -ForegroundColor Cyan
wsl --set-default $script:WslDistro

if ($LASTEXITCODE -ne 0) {
    throw "Failed to set Ubuntu as default WSL distro."
}

Write-Ok "Default WSL distro set to: $script:WslDistro"

Ensure-WslMirroredNetworking
Ensure-FirewallPort -Port 8765 -RuleName "TeLeRobot 8765"
Ensure-FirewallPort -Port 8080 -RuleName "TeLeRobot 8080"

Write-Step "Installing Windows launcher Python packages"

$PythonCmd = $script:PythonCmd
$WinReq = Join-Path $ProjectRootWin "install\requirements-windows.txt"

cmd /c "$PythonCmd -m pip install --upgrade pip"

if (Test-Path $WinReq) {
    cmd /c "$PythonCmd -m pip install -r `"$WinReq`""
} else {
    cmd /c "$PythonCmd -m pip install customtkinter pyyaml"
}

if ($LASTEXITCODE -ne 0) {
    throw "Windows launcher dependency install failed."
}

Write-Step "Preparing Ubuntu WSL system packages"

Write-Host "You may be asked for your Ubuntu sudo password."
Invoke-WslScript @'
set -e
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip v4l-utils openssl ffmpeg libgl1 libglib2.0-0
'@

Write-Step "Creating/updating WSL virtual environment from locked dependencies"

$SetupScriptTemplate = @'
set -e

python3 -m venv ~/telerobot-venv
source ~/telerobot-venv/bin/activate

python3 -m pip install --upgrade pip wheel 'setuptools>=71,<81'

cd '__PROJECT_ROOT_WSL__'

if [ -f install/requirements-wsl-lock.txt ]; then
  LOCK_FILE='install/requirements-wsl-lock.txt'
elif [ -f install/requirements-wsl-latest-lock.txt ]; then
  LOCK_FILE='install/requirements-wsl-latest-lock.txt'
else
  echo 'ERROR: No WSL requirements lock file found.'
  echo 'Expected install/requirements-wsl-lock.txt or install/requirements-wsl-latest-lock.txt.'
  exit 1
fi

echo "Installing exact pinned WSL dependency versions from $LOCK_FILE..."
python3 -m pip install --no-deps -r "$LOCK_FILE"

echo 'Installing TeLeRobot project in editable mode without dependency resolution...'
python3 -m pip install -e '__PROJECT_ROOT_WSL__' --no-deps

echo 'Verifying TeLeRobot package install...'
python3 -m pip show telerobot
python3 -c 'import telerobot; print(telerobot.__file__)'

echo 'Checking installed dependencies...'
PIP_CHECK_OUTPUT="$(python3 -m pip check 2>&1)" || PIP_CHECK_FAILED=1

if [ "${PIP_CHECK_FAILED:-0}" = "1" ]; then
  echo "$PIP_CHECK_OUTPUT"

  # Known metadata-only warning for this pinned LeRobot 0.5.1 + placo stack.
  # Installing coal-library causes slow/source builds and is not needed for the tested runtime.
  REAL_PIP_CHECK_ERRORS="$(echo "$PIP_CHECK_OUTPUT" | grep -v 'requires coal-library, which is not installed' || true)"

  if [ -n "$REAL_PIP_CHECK_ERRORS" ]; then
    echo 'ERROR: pip check found real dependency issues.'
    exit 1
  else
    echo 'WARNING: Ignored known coal-library metadata warning.'
  fi
else
  echo "$PIP_CHECK_OUTPUT"
fi
'@

$SetupScript = $SetupScriptTemplate.Replace("__PROJECT_ROOT_WSL__", $ProjectRootWsl)
Invoke-WslScript $SetupScript

Write-Step "Generating SSL certificate if needed"

$SslScript = @"
set -e
cd '$ProjectRootWsl'
mkdir -p ssl_cert

if [ ! -f ssl_cert/server.key ] || [ ! -f ssl_cert/server.crt ]; then
  openssl req -x509 -newkey rsa:4096 -keyout ssl_cert/server.key -out ssl_cert/server.crt -days 365 -nodes -subj '/CN=localhost'
  echo 'Generated ssl_cert/server.key and ssl_cert/server.crt'
else
  echo 'SSL certificate already exists.'
fi
"@

Invoke-WslScript $SslScript

Write-Step "Creating runtime files if missing"

$runtimeSettings = Join-Path $ProjectRootWin "runtime_settings.yaml"

if (-not (Test-Path $runtimeSettings)) {
@'
# Runtime teleop settings generated by setup
workspace_scale: "1:1 (Normal)"
gripper_trigger_mode: "Press Trigger to Close"
'@ | Set-Content -Encoding UTF8 $runtimeSettings

    Write-Ok "Created runtime_settings.yaml"
} else {
    Write-Ok "runtime_settings.yaml already exists"
}

$runtimeDir = Join-Path $ProjectRootWin "_runtime"

if (-not (Test-Path $runtimeDir)) {
    New-Item -ItemType Directory -Path $runtimeDir | Out-Null
    Write-Ok "Created _runtime folder"
}

Write-Step "Updating Run_UI.bat"

$batPath = Join-Path $ProjectRootWin "Run_UI.bat"

@'
@echo off
cd /d "%~dp0"

where pyw >nul 2>nul
if %errorlevel%==0 (
    start "" pyw -3 "%~dp0launcher\launcher.py"
    exit /b
)

where pythonw >nul 2>nul
if %errorlevel%==0 (
    start "" pythonw "%~dp0launcher\launcher.py"
    exit /b
)

echo Python launcher not found.
echo Install Python 3.11+ and make sure it is available from PATH.
pause
exit /b 1
'@ | Set-Content -Encoding ASCII $batPath

Write-Ok "Updated Run_UI.bat"

Write-Step "Verifying runtime imports"

Invoke-WslScript @'
set -e
source ~/telerobot-venv/bin/activate

PY_SITE=$(python3 -c 'import site; print(site.getsitepackages()[0])')
export LD_LIBRARY_PATH=$PY_SITE/cmeel.prefix/lib:$LD_LIBRARY_PATH

python3 -c 'import telerobot; import cv2; import yaml; import aiortc; import aiohttp; import scservo_sdk; import placo'
'@

Write-Step "Setup complete"

Write-Host ""
Write-Host "Next steps:" -ForegroundColor Green
Write-Host "1. Plug in the robot arm and cameras."
Write-Host "2. Run Run_UI.bat."
Write-Host "3. Click LAUNCH SERVER."
Write-Host "4. Open the printed https://<PC-IP>:8765 address in the Meta Quest browser."
Write-Host ""



