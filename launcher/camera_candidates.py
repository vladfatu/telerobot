import re


def clean_camera_friendly_name(name):
    """Clean duplicated/cluttered camera names from usbipd/V4L2 output."""
    name = str(name or "").strip()
    if not name:
        return ""

    parts = [p.strip() for p in name.split(",") if p.strip()]
    if len(parts) >= 2:
        normalized = [
            re.sub(r"\s+", " ", part).strip().lower()
            for part in parts
        ]

        if len(set(normalized)) == 1:
            return parts[-1]

    return re.sub(r"\s+", " ", name).strip()


def build_windows_camera_candidates(usbipd_out, is_usbipd_busid):
    """Build camera candidate list from Windows usbipd output before WSL attach."""
    cam_keywords = ["camera", "webcam", "video", "hd user facing", "capture"]
    candidates = []

    for line in (usbipd_out or "").splitlines():
        lower_line = line.lower()

        if any(kw in lower_line for kw in cam_keywords) and "ch34" not in lower_line:
            parts = line.split()

            if parts and is_usbipd_busid(parts[0]):
                busid = parts[0]

                # usbipd usually prints: BUSID VID:PID DEVICE STATE.
                name_parts = parts[2:-1] if len(parts) >= 4 else parts[1:]
                friendly = " ".join(name_parts).strip() or f"Camera BUSID {busid}"
                friendly = clean_camera_friendly_name(friendly)

                candidates.append(
                    {
                        "name": "wrist",
                        "busid": busid,
                        "usb_name": friendly,
                    }
                )

    return candidates
