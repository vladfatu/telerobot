import subprocess
import time


def ensure_usbipd_buses_shared(app, buses, label):
    """Ensure one or more USB devices are shared with usbipd."""
    if not buses:
        return

    if isinstance(buses, str):
        buses = [buses]

    buses = [bus for bus in buses if bus]
    if not buses:
        return

    needs_elevation = []

    for bus in buses:
        bind_res = subprocess.run(
            ["usbipd", "bind", "--busid", bus],
            capture_output=True,
            text=True,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )

        bind_text = ((bind_res.stdout or "") + "\n" + (bind_res.stderr or "")).strip()
        bind_lower = bind_text.lower()

        if bind_res.returncode == 0:
            app.log_to_terminal(f"✨ {label} BUSID {bus} shared with usbipd.\n")
            continue

        if (
            "already" in bind_lower
            or "shared" in bind_lower
            or "bound" in bind_lower
            or "attached" in bind_lower
        ):
            continue

        needs_admin = (
            "administrator" in bind_lower
            or "access is denied" in bind_lower
            or "elevated" in bind_lower
            or "not shared" in bind_lower
        )

        if not needs_admin:
            app.log_to_terminal(f"⚠️ Could not share {label} BUSID {bus} normally: {bind_text}\n")

        needs_elevation.append(bus)

    if not needs_elevation:
        return

    unique_buses = []
    for bus in needs_elevation:
        if bus not in unique_buses:
            unique_buses.append(bus)

    app.log_to_terminal(
        f">> Administrator permission is needed to share one or more {label.lower()} device(s) with usbipd.\n"
    )
    app.log_to_terminal(
        ">> A Windows UAC prompt may appear. Click Yes to allow USB device sharing.\n"
    )

    safe_label = str(label).lower().replace(" ", "_")
    ps1_path = app.runtime_dir / f"usbipd_bind_{safe_label}_admin.ps1"

    commands = [
        "$ErrorActionPreference = 'Continue'",
        "Write-Host ''",
        f"Write-Host 'Telerobot {label} sharing setup' -ForegroundColor Cyan",
        "Write-Host 'This window is running as Administrator.' -ForegroundColor Yellow",
        "Write-Host ''",
    ]

    for bus in unique_buses:
        commands.extend(
            [
                f"Write-Host 'Sharing {label} BUSID {bus} with usbipd...'",
                f"usbipd bind --busid {bus}",
                "Write-Host ''",
            ]
        )

    commands.extend(
        [
            f"Write-Host '{label} sharing setup complete.' -ForegroundColor Green",
            "Start-Sleep -Seconds 1",
        ]
    )

    ps1_path.write_text("\n".join(commands), encoding="utf-8")

    ps_arg = f'-NoProfile -ExecutionPolicy Bypass -File "{ps1_path}"'

    elevate_res = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            f"Start-Process PowerShell -Verb RunAs -Wait -ArgumentList '{ps_arg}'",
        ],
        capture_output=True,
        text=True,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )

    elevate_text = ((elevate_res.stdout or "") + "\n" + (elevate_res.stderr or "")).strip()

    if elevate_res.returncode != 0:
        app.log_to_terminal(
            f"⚠️ Administrator {label} sharing step did not complete successfully: {elevate_text}\n"
        )
        return

    for bus in unique_buses:
        verify_res = subprocess.run(
            ["usbipd", "bind", "--busid", bus],
            capture_output=True,
            text=True,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )

        verify_text = ((verify_res.stdout or "") + "\n" + (verify_res.stderr or "")).strip()
        verify_lower = verify_text.lower()

        if (
            verify_res.returncode == 0
            or "already" in verify_lower
            or "shared" in verify_lower
            or "bound" in verify_lower
            or "attached" in verify_lower
        ):
            app.log_to_terminal(f"✨ {label} BUSID {bus} is now shared with usbipd.\n")
        else:
            app.log_to_terminal(f"⚠️ {label} BUSID {bus} still does not appear shared: {verify_text}\n")


def attach_usbipd_device_to_wsl(app, busid, label, attempts=3):
    """Detach then attach one usbipd device to WSL with retries."""
    subprocess.run(
        ["usbipd", "detach", "--busid", busid],
        capture_output=True,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )
    time.sleep(0.8)

    attach_text = ""

    for attempt in range(1, attempts + 1):
        attach_res = subprocess.run(
            ["usbipd", "attach", "--wsl", "--busid", busid],
            capture_output=True,
            text=True,
            timeout=15,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )

        attach_text = ((attach_res.stdout or "") + "\n" + (attach_res.stderr or "")).strip()

        if attach_res.returncode == 0 or "already attached" in attach_text.lower():
            app.log_to_terminal(f"✨ Successfully attached {label} BUSID {busid} to WSL!\n")
            return True, attach_text

        if attempt < attempts:
            app.log_to_terminal(f"⚠️ {label} BUSID {busid} attach attempt {attempt} failed. Retrying...\n")
            time.sleep(2.0)

    return False, attach_text


def find_wsl_serial_port(app, attempts=12, delay_s=1.0):
    """Find the first WSL serial port for the robot arm."""
    for _ in range(attempts):
        try:
            res = subprocess.run(
                [
                    "wsl",
                    "--",
                    "bash",
                    "-lc",
                    "ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1 || true",
                ],
                capture_output=True,
                text=True,
                timeout=5,
                creationflags=subprocess.CREATE_NO_WINDOW,
            )

            port = (res.stdout or "").strip().splitlines()
            if port:
                detected = port[0].strip()

                subprocess.run(
                    ["wsl", "-u", "root", "chmod", "666", detected],
                    creationflags=subprocess.CREATE_NO_WINDOW,
                )

                return detected

        except Exception:
            pass

        time.sleep(delay_s)

    return None
