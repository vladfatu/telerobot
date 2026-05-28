"""Log filtering rules for the launcher terminal."""


ALWAYS_HIDDEN_KEYWORDS = [
    "Incoming request",
    "Response: 200",
    "aiohttp.access",
    "GET /",
]


NORMAL_MODE_HIDDEN_KEYWORDS = [
    # Detailed USB/camera startup noise.
    "✨ Arm BUSID",
    "✨ Successfully attached Arm BUSID",
    "✨ Auto-detected Arm Port:",
    "✨ Camera BUSID",
    "✨ Successfully attached Camera BUSID",
    "attach attempt",
    "Pausing for 5 seconds",
    "Unlocking Camera Permissions",
    "OpenCV-validated camera devices detected",
    "(MJPG 640x480@30)",
    "Camera configured: OpenCV-validated MJPG",

    # Runtime settings updates.
    "⚙️ Runtime controls updated:",
    "⚙️ VR runtime controls updated:",
    "⚙️ Runtime settings sent:",

    # Admin/UAC setup details.
    "Administrator permission is needed",
    "A Windows UAC prompt may appear",

    # Reset details.
    "Soft resetting robot to initial position",
    "Soft reset skipped",
    "Soft reset complete",
    "Soft reset stopped",

    # WebRTC connection details.
    "✅ WebRTC camera connected:",

    # Camera decoder noise from MJPG streams.
    "Corrupt JPEG data",
    "premature end of data segment",
    "found marker",
    "extraneous bytes before marker",

    # Server/network internals.
    "✅ SSL enabled with",
    "✅ SSL enabled for WebSocket server",
    "🚀 Starting websocket server thread",
    "🌐 WebSocket server listening on",
    "✅ WebSocket server is ready",
    "WebRTC Camera Server started",
    "server listening on",
    "Returning WebRTC answer",
    "Connection state for",
    "ICE gathering state",
    "aioice.ice",
    "Check CandidatePair",
]


def should_hide_log_line(clean_text, debug_enabled=False):
    """Return True if a terminal line should be hidden."""
    if any(spam in clean_text for spam in ALWAYS_HIDDEN_KEYWORDS):
        return True

    if not debug_enabled and any(spam in clean_text for spam in NORMAL_MODE_HIDDEN_KEYWORDS):
        return True

    return False
