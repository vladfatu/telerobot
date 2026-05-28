import logging
import time

TIMING_INTERVAL = 100  # Log detailed timing every N loops


def get_logger() -> logging.Logger:
    logger = logging.getLogger("telerobot.cli")
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger


def log_message(logger: logging.Logger, message: str) -> None:
    logger.info(message)


def maybe_log_loop_timing(
    logger: logging.Logger,
    loop_count: int,
    t0: float,
    t_control: float | None,
    t_rerun: float | None,
    t_camera: float,
    camera_stats: dict | None = None,
) -> None:
    if loop_count % TIMING_INTERVAL != 0:
        return

    total = time.perf_counter() - t0
    parts = []
    if t_control is not None:
        parts.append(f"control+IK: {(t_control - t0) * 1000:.1f}ms")
    if t_rerun is not None and t_control is not None:
        parts.append(f"rerun: {(t_rerun - t_control) * 1000:.1f}ms")

    camera_start = t_rerun or t_control or t0

    # This is NOT physical camera FPS. It is only the cost of forwarding/updating
    # the latest frame in the loop, so call it frame update instead of camera stream.
    parts.append(f"frame update: {(t_camera - camera_start) * 1000:.1f}ms")

    if camera_stats:
        summaries = []
        for name, stat in camera_stats.items():
            interval = stat.get("interval_ema")
            if interval and interval > 0:
                fps = 1.0 / interval
                ms = interval * 1000.0
                summaries.append(f"{name}: {fps:.1f}fps/{ms:.0f}ms")

        if summaries:
            parts.append("camera estimate: " + ", ".join(summaries))

    parts.append(f"total: {total * 1000:.1f}ms")
    parts.append(f"loop fps: {1 / total:.1f}")
    logger.info(f"⏱ Loop {loop_count} | " + " | ".join(parts))
