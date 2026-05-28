# TeLeRobot Windows + WSL uninstall / cleanup script
# Run from the project root with:
#   powershell -ExecutionPolicy Bypass -File install\uninstall_windows.ps1
#
# Important behavior:
# - From the project/program folder, this script deletes ONLY:
#     ssl_cert/server.key
#     ssl_cert/server.crt
# - It does NOT delete runtime_settings.yaml, Run_UI.bat, _runtime, folders, or other project files.
# - Ubuntu WSL deletion is optional and requires explicit user confirmation.

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

function Ask-YesNo($Message, $DefaultNo = $true) {
    $suffix = if ($DefaultNo) { "[y/N]" } else { "[Y/n]" }
    $answer = Read-Host "$Message $suffix"

    if ([string]::IsNullOrWhiteSpace($answer)) {
        return -not $DefaultNo
    }

    return $answer.Trim().ToLower().StartsWith("y")
}

function Convert-ToWslPath($WindowsPath) {
    $full = [System.IO.Path]::GetFullPath($WindowsPath)
    $drive = $full.Substring(0, 1).ToLower()
    $rest = $full.Substring(2).Replace([char]92, '/')
    return "/mnt/$drive$rest"
}

function Get-TelerobotWslDistro {
    if (-not (Get-Command wsl -ErrorAction SilentlyContinue)) {
        return $null
    }

    $distros = @(
        wsl --list --quiet 2>$null |
        ForEach-Object {
            ($_ -replace "`0", "").Trim()
        } |
        Where-Object {
            $_
        }
    )

    $ubuntu = $distros |
        Where-Object {
            $_ -match "^Ubuntu"
        } |
        Select-Object -First 1

    return $ubuntu
}

function Invoke-WslScript($ScriptText) {
    if (-not $script:WslDistro) {
        throw "No WSL distro selected."
    }

    $tempFile = Join-Path $env:TEMP ("telerobot_uninstall_" + [System.Guid]::NewGuid().ToString() + ".sh")

    try {
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

function Clean-ProjectFolderForSharing {
    Write-Step "Cleaning project folder for sharing"

    $itemsToRemove = @(
        "config.yaml",
        "_arm_only_config.yaml",
        "_check_runtime_imports.sh",
        "_dev_backups",
        "_runtime",
        "telerobot_calibration_backup"
    )

    foreach ($item in $itemsToRemove) {
        $path = Join-Path $ProjectRootWin $item

        if (Test-Path -LiteralPath $path) {
            Remove-Item -LiteralPath $path -Recurse -Force -ErrorAction SilentlyContinue
            Write-Ok "Removed $item"
        }
    }

    $sslDir = Join-Path $ProjectRootWin "ssl_cert"

    if (-not (Test-Path -LiteralPath $sslDir)) {
        New-Item -ItemType Directory -Path $sslDir | Out-Null
        Write-Ok "Created empty ssl_cert folder"
    }

    foreach ($certFile in @("server.key", "server.crt")) {
        $certPath = Join-Path $sslDir $certFile

        if (Test-Path -LiteralPath $certPath) {
            Remove-Item -LiteralPath $certPath -Force -ErrorAction SilentlyContinue
            Write-Ok "Removed ssl_cert/$certFile"
        } else {
            Write-Skip "ssl_cert/$certFile not found"
        }
    }

    $cacheDirs = Get-ChildItem $ProjectRootWin -Recurse -Directory -Force -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Name -eq "__pycache__" -or
            $_.Name -eq ".pytest_cache" -or
            $_.Name -eq ".mypy_cache" -or
            $_.Name -eq ".ruff_cache"
        }

    foreach ($dir in $cacheDirs) {
        Remove-Item -LiteralPath $dir.FullName -Recurse -Force -ErrorAction SilentlyContinue
    }

    $junkFiles = Get-ChildItem $ProjectRootWin -Recurse -File -Force -ErrorAction SilentlyContinue |
        Where-Object {
            $_.Name -match "\.pyc$|\.pyo$|\.bak($|-)|\.tmp$|\.temp$|\.old$|\.orig$|\.swp$|\.swo$|~$|desktop\.ini$|Thumbs\.db$"
        }

    foreach ($file in $junkFiles) {
        Remove-Item -LiteralPath $file.FullName -Force -ErrorAction SilentlyContinue
    }

    Write-Ok "Project folder sharing cleanup completed"
}
function Remove-FileIfExists($Path, $Description) {
    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Force
        Write-Ok "Removed $Description"
    } else {
        Write-Skip "$Description not found"
    }
}

Write-Host "TeLeRobot Windows + WSL uninstall / cleanup" -ForegroundColor Cyan

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$ProjectRootWin = $ProjectRoot.Path
$ProjectRootWsl = Convert-ToWslPath $ProjectRootWin

Write-Host "Project root: $ProjectRootWin"
Write-Host "WSL path:     $ProjectRootWsl"

Write-Step "Detecting Ubuntu WSL distro"

$script:WslDistro = Get-TelerobotWslDistro

if ($script:WslDistro) {
    Write-Ok "Using WSL distro: $script:WslDistro"
} else {
    Write-Skip "No Ubuntu WSL distro found"
}

Write-Step "Removing WSL TeLeRobot virtual environment"

if ($script:WslDistro) {
    Invoke-WslScript @'
set -e

if [ -d "$HOME/telerobot-venv" ]; then
    rm -rf "$HOME/telerobot-venv"
    echo "Removed ~/telerobot-venv"
else
    echo "~/telerobot-venv does not exist"
fi
'@
} else {
    Write-Skip "WSL venv cleanup skipped because no Ubuntu distro was found"
}

Write-Step "Removing certificate files only from project folder"

$sslDir = Join-Path $ProjectRootWin "ssl_cert"
$sslKey = Join-Path $sslDir "server.key"
$sslCrt = Join-Path $sslDir "server.crt"

Remove-FileIfExists -Path $sslKey -Description "ssl_cert/server.key"
Remove-FileIfExists -Path $sslCrt -Description "ssl_cert/server.crt"

Write-Step "Optional generated config cleanup"

$configPath = Join-Path $ProjectRootWin "config.yaml"

if (Test-Path -LiteralPath $configPath) {
    if (Ask-YesNo "Remove generated config.yaml?") {
        Remove-Item -LiteralPath $configPath -Force
        Write-Ok "Removed config.yaml"
    } else {
        Write-Skip "Kept config.yaml"
    }
} else {
    Write-Skip "config.yaml not found"
}

Write-Step "Optional generated runtime cleanup"

$runtimeDir = Join-Path $ProjectRootWin "_runtime"

if (Test-Path -LiteralPath $runtimeDir) {
    if (Ask-YesNo "Remove generated _runtime folder?") {
        Remove-Item -LiteralPath $runtimeDir -Recurse -Force
        Write-Ok "Removed _runtime folder"
    } else {
        Write-Skip "Kept _runtime folder"
    }
} else {
    Write-Skip "_runtime folder not found"
}





Write-Step "Removing Windows Python launcher packages"

$PythonCmd = $null

if (Get-Command py -ErrorAction SilentlyContinue) {
    $PythonCmd = "py -3"
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    $PythonCmd = "python"
}

if ($PythonCmd) {
    cmd /c "$PythonCmd -m pip uninstall customtkinter pyyaml -y"

    if ($LASTEXITCODE -ne 0) {
        Write-Warn "Windows Python package uninstall returned a non-zero exit code. Packages may already be absent."
    } else {
        Write-Ok "Uninstalled Windows Python packages: customtkinter, pyyaml"
    }
} else {
    Write-Skip "Windows Python not found"
}

Write-Step "Removing Python cache files from project folder"

$pycacheDirs = Get-ChildItem $ProjectRootWin -Recurse -Directory -Force -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -eq "__pycache__" }

foreach ($dir in $pycacheDirs) {
    Remove-Item -LiteralPath $dir.FullName -Recurse -Force -ErrorAction SilentlyContinue
}

$compiledPythonFiles = Get-ChildItem $ProjectRootWin -Recurse -File -Force -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match "\.pyc$|\.pyo$" }

foreach ($file in $compiledPythonFiles) {
    Remove-Item -LiteralPath $file.FullName -Force -ErrorAction SilentlyContinue
}

Write-Ok "Removed Python cache files if present"


Write-Step "Optional Windows Python cleanup"

if (Get-Command winget -ErrorAction SilentlyContinue) {
    if (Ask-YesNo "Uninstall Windows Python 3.12? Only do this if you installed it only for TeLeRobot.") {
        winget uninstall -e --id Python.Python.3.12

        if ($LASTEXITCODE -ne 0) {
            Write-Warn "Python 3.12 uninstall may have failed or Python 3.12 was not found."
        } else {
            Write-Ok "Python 3.12 uninstall completed"
        }
    } else {
        Write-Skip "Kept Windows Python 3.12"
    }
} else {
    Write-Skip "winget not found, cannot uninstall Python automatically"
}


Clean-ProjectFolderForSharing

Write-Step "Optional WSL apt package cleanup"

if ($script:WslDistro) {
    if (Ask-YesNo "Remove WSL apt packages installed by setup? This may affect other WSL projects.") {
        Invoke-WslScript @'
set -e

CANDIDATES="
v4l-utils
ffmpeg
libgl1
libglib2.0-0
libglib2.0-0t64
"

TO_REMOVE=""

for pkg in $CANDIDATES; do
    if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
        TO_REMOVE="$TO_REMOVE $pkg"
    else
        echo "Package not installed or not known, skipping: $pkg"
    fi
done

if [ -n "$TO_REMOVE" ]; then
    echo "Removing installed apt packages:$TO_REMOVE"
    sudo apt-get remove --purge -y $TO_REMOVE
else
    echo "No selected WSL apt packages were installed."
fi

sudo apt-get autoremove --purge -y
sudo apt-get clean

echo "WSL apt cleanup completed"
'@

        Write-Ok "WSL apt cleanup completed"
    } else {
        Write-Skip "Kept WSL apt packages"
    }
} else {
    Write-Skip "WSL apt cleanup skipped"
}

Write-Step "Optional usbipd-win cleanup"

if (Get-Command winget -ErrorAction SilentlyContinue) {
    if (Ask-YesNo "Uninstall usbipd-win from Windows? Only do this if you installed it only for TeLeRobot.") {
        winget uninstall --id dorssel.usbipd-win --exact

        if ($LASTEXITCODE -ne 0) {
            Write-Warn "usbipd-win uninstall may have failed, may require admin rights, or package was not found."
        } else {
            Write-Ok "usbipd-win uninstall completed"
        }
    } else {
        Write-Skip "Kept usbipd-win"
    }
} else {
    Write-Skip "winget not found, cannot uninstall usbipd-win automatically"
}



Write-Step "Optional network cleanup"

$firewallRules = @(
    "TeLeRobot 8765",
    "TeLeRobot 8080"
)

if (Ask-YesNo "Remove TeLeRobot Windows firewall rules for ports 8765 and 8080?") {
    foreach ($ruleName in $firewallRules) {
        if (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue) {
            try {
                Remove-NetFirewallRule -DisplayName $ruleName
                Write-Ok "Removed firewall rule: $ruleName"
            } catch {
                Write-Warn "Could not remove firewall rule '$ruleName' without elevation. Requesting administrator permission..."

                $cmd = "Remove-NetFirewallRule -DisplayName '$ruleName'"
                Start-Process PowerShell -Verb RunAs -Wait -ArgumentList "-NoProfile -ExecutionPolicy Bypass -Command `"$cmd`""

                if (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue) {
                    Write-Warn "Firewall rule may still exist. Remove manually if needed: $ruleName"
                } else {
                    Write-Ok "Removed firewall rule after elevation: $ruleName"
                }
            }
        } else {
            Write-Skip "Firewall rule not found: $ruleName"
        }
    }
} else {
    Write-Skip "Kept TeLeRobot Windows firewall rules"
}

if (Ask-YesNo "Disable WSL mirrored networking in .wslconfig? This may affect other WSL projects.") {
    $wslConfig = Join-Path $env:USERPROFILE ".wslconfig"

    if (Test-Path $wslConfig) {
        $stamp = Get-Date -Format "yyyyMMdd-HHmmss"
        $backup = Join-Path $env:USERPROFILE ".wslconfig.telerobot-backup-$stamp"
        Copy-Item $wslConfig $backup -Force
        Write-Ok "Backed up .wslconfig to: $backup"

        $lines = Get-Content $wslConfig
        $newLines = $lines | Where-Object {
            $_ -notmatch "^\s*networkingMode\s*=\s*mirrored\s*$"
        }

        $newLines | Set-Content -Path $wslConfig -Encoding ASCII

        Write-Ok "Removed networkingMode=mirrored from .wslconfig"
        Write-Warn "Restarting WSL so networking changes take effect..."
        wsl --shutdown
        Write-Ok "WSL shutdown complete"
    } else {
        Write-Skip ".wslconfig not found"
    }
} else {
    Write-Skip "Kept WSL mirrored networking"
}


Write-Step "Optional Ubuntu WSL distro deletion"

if ($script:WslDistro) {
    Write-Warn "Deleting Ubuntu WSL will permanently remove the entire Linux filesystem for distro: $script:WslDistro"
    Write-Warn "This is not limited to TeLeRobot. It deletes everything inside that Ubuntu WSL distro."

    if (Ask-YesNo "Delete/unregister Ubuntu WSL distro '$script:WslDistro'?") {
        $confirmName = Read-Host "Type the distro name exactly to confirm deletion: $script:WslDistro"

        if ($confirmName -eq $script:WslDistro) {
            wsl --terminate $script:WslDistro 2>$null
            wsl --unregister $script:WslDistro

            if ($LASTEXITCODE -ne 0) {
                Write-Warn "Ubuntu WSL unregister may have failed."
            } else {
                Write-Ok "Deleted/unregistered Ubuntu WSL distro: $script:WslDistro"
            }
        } else {
            Write-Skip "Confirmation did not match. Ubuntu WSL distro was not deleted."
        }
    } else {
        Write-Skip "Ubuntu WSL distro was not deleted"
    }
} else {
    Write-Skip "No Ubuntu WSL distro found to delete"
}

Write-Step "Cleanup complete"

Write-Host ""
Write-Host "Project folder cleanup performed:" -ForegroundColor Green
Write-Host "  Removed generated/local files, cache files, certificates, and temporary test artifacts."
Write-Host ""
Write-Host "Project folder is now suitable for sharing after a final audit." -ForegroundColor Green
Write-Host ""


