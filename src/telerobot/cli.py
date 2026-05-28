import argparse
import sys
import builtins
import logging
import os
import time
import yaml

import cv2
import numpy as np
from pathlib import Path

from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from telerobot.config import load_robot
from telerobot.controller import build_controller
from telerobot.dataset import setup_dataset, end_active_episode, record_step, finalize_dataset
from telerobot.logger import get_logger, log_message, maybe_log_loop_timing
from telerobot.server import setup_webxr_server, setup_websocket_server

# Resolve default config path relative to the project root
PROJECT_ROOT = Path(__file__).parent.parent.parent
DEFAULT_CONFIG_PATH = str(PROJECT_ROOT / "config.yaml")
RUNTIME_SETTINGS_PATH = PROJECT_ROOT / "runtime_settings.yaml"


def make_mock_camera_frame(camera_name: str, loop_count: int, width: int = 640, height: int = 480):
    """Generate a temporary synthetic RGB frame for multi-camera testing."""
    palette = {
        "wrist": (40, 80, 220),
        "overhead": (40, 180, 80),
        "side": (220, 120, 40),
        "front": (180, 80, 200),
        "rear": (180, 180, 60),
    }

    color = palette.get(camera_name, (100, 100, 100))

    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = color

    # Add simple motion so it is obvious the stream is live.
    x = int((loop_count * 7) % max(width - 80, 1))
    y = int(height * 0.65)
    cv2.rectangle(frame, (x, y), (x + 80, y + 50), (255, 255, 255), -1)

    label = f"MOCK CAMERA: {camera_name}"
    counter = f"Loop: {loop_count}"

    cv2.putText(frame, label, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, counter, (30, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    return frame


def update_camera_fps_estimate(camera_stats: dict, camera_name: str, frame):
    """Estimate real camera update rate by detecting frame content changes.

    This is intentionally lightweight: it samples a sparse grid of pixels and
    tracks the time between content changes. It is an estimate, not a hardware
    timestamp, but it is far closer to physical camera FPS than loop timing.
    """
    if frame is None:
        return

    try:
        arr = np.asarray(frame)
        if arr.ndim < 2:
            return

        h, w = arr.shape[:2]
        y_step = max(1, h // 12)
        x_step = max(1, w // 12)

        sample = arr[::y_step, ::x_step]
        checksum = int(np.sum(sample, dtype=np.uint64) % 1000000007)
    except Exception:
        return

    now = time.perf_counter()
    stat = camera_stats.setdefault(
        camera_name,
        {
            "last_checksum": None,
            "last_change_time": None,
            "interval_ema": None,
            "changes": 0,
        },
    )

    if stat["last_checksum"] is None:
        stat["last_checksum"] = checksum
        stat["last_change_time"] = now
        return

    if checksum == stat["last_checksum"]:
        return

    last_time = stat.get("last_change_time")
    if last_time is not None:
        interval = now - last_time
        if 0.001 <= interval <= 5.0:
            if stat["interval_ema"] is None:
                stat["interval_ema"] = interval
            else:
                # Smooth the estimate so it does not jump too much.
                stat["interval_ema"] = stat["interval_ema"] * 0.8 + interval * 0.2

    stat["last_checksum"] = checksum
    stat["last_change_time"] = now
    stat["changes"] += 1



def precision_mode_to_runtime_scales(mode: str) -> tuple[float, float]:
    """Convert launcher workspace scale label into live position/rotation scales."""
    mode = (mode or "").lower()

    if mode.startswith("2:1") or "delicate" in mode:
        return 0.5, 0.5

    return 1.0, 1.0


def load_runtime_settings(path: Path) -> dict:
    """Load runtime teleop settings from YAML. Returns empty dict if unavailable."""
    try:
        if not path.exists():
            return {}

        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            return {}

        return data
    except Exception:
        return {}

def main():
    logger = get_logger()

    parser = argparse.ArgumentParser(description="Telerobot — VR teleoperation for SO-ARM101")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )
    # --- NEW: DUMMY FLAG ---
    parser.add_argument(
        "--dummy",
        action="store_true",
        help="Run in dummy mode without requiring physical robot connection",
    )
    parser.add_argument(
        "--calibrate-only",
        action="store_true",
        help="Connect and calibrate the robot, then exit without starting teleoperation/server.",
    )

    args = parser.parse_args()

    debug_logs = os.getenv("TELEROBOT_DEBUG", "0") == "1"
    mock_cameras = os.getenv("TELEROBOT_MOCK_CAMERAS", "0") == "1"

    logging.basicConfig(
        level=logging.INFO if debug_logs else logging.WARNING,
        format="%(levelname)s:%(name)s:%(message)s",
        force=True,
    )

    if not debug_logs:
        for noisy_logger in [
            "aioice",
            "aioice.ice",
            "aiortc",
            "websockets",
            "websockets.server",
            "aiohttp",
            "aiohttp.access",
            "telerobot.server.webxr_server",
            "lerobot",
            "lerobot.cameras.opencv.camera_opencv",
            "lerobot.robots.so_follower.so_follower",
        ]:
            logging.getLogger(noisy_logger).setLevel(logging.WARNING)

    duo_robot, cfg = load_robot(args.config)

    camera_server = setup_webxr_server(duo_robot, logger, dataset_configured=cfg.dataset is not None)
    teleop_device = setup_websocket_server()

    controller = build_controller(duo_robot, cfg)
    dataset = setup_dataset(duo_robot, cfg, logger)

    # --- DUMMY MODE HARDWARE BYPASS ---
    if args.dummy:
        log_message(logger, "⚠️ RUNNING IN DUMMY MODE: Bypassing physical robot connection.")
        if mock_cameras:
            log_message(logger, "🎥 MOCK CAMERA MODE: Generating synthetic multi-camera streams.")
    else:
        pass

    original_input = builtins.input

    def auto_accept_existing_calibration(prompt=""):
        if "Press ENTER to use provided calibration file" in str(prompt):
            print(str(prompt))
            print("Auto-accepting existing calibration file in non-interactive launcher mode.")
            return ""
        return original_input(prompt)

    if not getattr(args, "calibrate_only", False) and not sys.stdin.isatty():
        builtins.input = auto_accept_existing_calibration

    try:
        duo_robot.connect()
    finally:
        builtins.input = original_input

    if getattr(args, "calibrate_only", False):
        print("Calibration-only mode complete. Exiting before server/teleop starts.")
        try:
            duo_robot.disconnect()
        except Exception:
            pass
        return

        if not duo_robot.is_connected or not teleop_device.is_connected:
            raise ValueError("Robot or teleop is not connected!")

    if cfg.use_rerun:
        init_rerun(session_name="vr_lerobot_teleop")

    try:
        controller.capture_initial_observations()
    except Exception as e:
        if args.dummy:
            log_message(logger, "Dummy Mode: Skipped capturing initial hardware observations.")
        else:
            raise e

    recording = False
    finalized_dataset = False
    push_to_hub = cfg.dataset.push_to_hub if cfg.dataset else False

    # Dummy mode: connect cameras manually because robot.connect() is skipped.
    if getattr(args, "dummy", False) and not mock_cameras and getattr(duo_robot, "cameras", None):
        for cam_name, cam in duo_robot.cameras.items():
            try:
                if hasattr(cam, "is_connected") and not cam.is_connected:
                    log_message(logger, f"Dummy Mode: connecting camera '{cam_name}'...")
                    cam.connect(warmup=True)
                    log_message(logger, f"Dummy Mode: camera '{cam_name}' connected.")
            except Exception as e:
                log_message(logger, f"Dummy Mode: failed to connect camera '{cam_name}': {type(e).__name__}: {e}")

    log_message(logger, "Starting teleop loop. Connect your VR headset to teleoperate the robot...")
    loop_count = 0
    camera_fps_stats = {}
    last_reset_time = 0.0
    reset_action_active = False
    runtime_settings_mtime = None
    last_runtime_settings_check = 0.0
    last_vr_runtime_settings = None

    try:
        while True:
            t0 = time.perf_counter()
            t_control = t_rerun = t_dataset = None

            # Poll runtime teleop settings without reconnecting hardware/cameras.
            now_runtime_check = time.perf_counter()
            if now_runtime_check - last_runtime_settings_check >= 0.5:
                last_runtime_settings_check = now_runtime_check

                try:
                    current_mtime = RUNTIME_SETTINGS_PATH.stat().st_mtime if RUNTIME_SETTINGS_PATH.exists() else None
                except Exception:
                    current_mtime = None

                if current_mtime is not None and current_mtime != runtime_settings_mtime:
                    runtime_settings_mtime = current_mtime
                    runtime_settings = load_runtime_settings(RUNTIME_SETTINGS_PATH)

                    workspace_scale = runtime_settings.get("workspace_scale", getattr(cfg.teleop, "precision_mode", "1:1 (Normal)"))
                    position_scale, rotation_scale = precision_mode_to_runtime_scales(workspace_scale)
                    gripper_trigger_mode = runtime_settings.get(
                        "gripper_trigger_mode",
                        getattr(cfg.teleop, "gripper_trigger_mode", "Press Trigger to Close"),
                    )

                    if controller.update_runtime_settings(
                        position_scale=position_scale,
                        rotation_scale=rotation_scale,
                        gripper_trigger_mode=gripper_trigger_mode,
                    ):
                        log_message(
                            logger,
                            f"⚙️ Runtime controls updated: {workspace_scale}, {gripper_trigger_mode}",
                        )

            # Apply runtime settings sent from the VR/web UI.
            vr_runtime_settings = getattr(teleop_device, "last_runtime_settings", None)
            if isinstance(vr_runtime_settings, dict):
                vr_runtime_tuple = (
                    vr_runtime_settings.get("workspace_scale"),
                    vr_runtime_settings.get("gripper_trigger_mode"),
                )

                if vr_runtime_tuple != last_vr_runtime_settings:
                    workspace_scale = vr_runtime_settings.get("workspace_scale", getattr(cfg.teleop, "precision_mode", "1:1 (Normal)"))
                    gripper_trigger_mode = vr_runtime_settings.get(
                        "gripper_trigger_mode",
                        getattr(cfg.teleop, "gripper_trigger_mode", "Press Trigger to Close"),
                    )

                    position_scale, rotation_scale = precision_mode_to_runtime_scales(workspace_scale)

                    if controller.update_runtime_settings(
                        position_scale=position_scale,
                        rotation_scale=rotation_scale,
                        gripper_trigger_mode=gripper_trigger_mode,
                    ):
                        log_message(
                            logger,
                            f"⚙️ VR runtime controls updated: {workspace_scale}, {gripper_trigger_mode}",
                        )

                    # Keep runtime_settings.yaml in sync so launcher/server state agrees.
                    try:
                        RUNTIME_SETTINGS_PATH.write_text(
                            (
                                "# Runtime teleop settings generated by VR/web UI\n"
                                f"workspace_scale: \"{workspace_scale}\"\n"
                                f"gripper_trigger_mode: \"{gripper_trigger_mode}\"\n"
                            ),
                            encoding="utf-8",
                        )
                        runtime_settings_mtime = RUNTIME_SETTINGS_PATH.stat().st_mtime
                    except Exception as e:
                        if os.getenv("TELEROBOT_DEBUG", "0") == "1":
                            log_message(logger, f"Failed to sync VR runtime settings file: {type(e).__name__}: {e}")

                    last_vr_runtime_settings = vr_runtime_tuple

            # Get teleop action
            vr_obs = teleop_device.last_observation
            camera_frames = {}




            if vr_obs is None:
                pass
            else:
                action_str = vr_obs.get('action', 'none')

                if action_str == 'reset':
                    now_reset = time.perf_counter()

                    # Safety guard:
                    # Accept reset only on the rising edge of a reset action,
                    # and never more often than once every 8 seconds.
                    reset_allowed = (
                        not reset_action_active
                        and not controller.has_initial_position
                        and (now_reset - last_reset_time) > 8.0
                    )

                    reset_action_active = True

                    if reset_allowed:
                        last_reset_time = now_reset
                        if not args.dummy:
                            if hasattr(controller, "soft_reset"):
                                controller.soft_reset()
                            else:
                                controller.reset()
                    else:
                        if os.getenv("TELEROBOT_DEBUG", "0") == "1":
                            log_message(logger, "Ignored repeated/reset-cooldown reset action.")
                elif action_str == 'start_episode' and not recording:
                    log_message(logger, f"🔴 Recording episode {dataset.num_episodes if dataset else '?'}...")
                    recording = True
                    finalized_dataset = False
                elif action_str == 'stop_episode' and recording:
                    if not args.dummy:
                        if hasattr(controller, "soft_reset"):
                            controller.soft_reset()
                        else:
                            controller.reset()
                    end_active_episode(dataset, logger)
                    recording = False
                elif action_str == 'save_dataset' and not finalized_dataset:
                    finalize_dataset(dataset, push_to_hub, logger)
                    recording = False
                    finalized_dataset = True
                else:
                    if action_str != 'reset':
                        reset_action_active = False

                    if not args.dummy:
                        result = controller.process_vr_observation(vr_obs)
                        t_control = time.perf_counter()

                        if result is not None:
                            obs, action = result
                            for cam_name in duo_robot.cameras:
                                if cam_name in obs:
                                    camera_frames[cam_name] = obs[cam_name]

                            if cfg.use_rerun:
                                log_rerun_data(observation=obs, action=action)

                            t_rerun = time.perf_counter()

                            if recording:
                                record_step(dataset, cfg, obs, action)

            # Camera streaming
            if mock_cameras:
                for cam_name in duo_robot.cameras.keys():
                    frame = make_mock_camera_frame(cam_name, loop_count)
                    update_camera_fps_estimate(camera_fps_stats, cam_name, frame)
                    camera_server.update_camera_frame(cam_name, frame)
            else:
                for cam_name, cam in duo_robot.cameras.items():
                    try:
                        if hasattr(cam, "is_connected") and not cam.is_connected:
                            continue

                        frame = camera_frames.get(cam_name) if camera_frames else cam.async_read(timeout_ms=15000)
                        update_camera_fps_estimate(camera_fps_stats, cam_name, frame)
                        camera_server.update_camera_frame(cam_name, frame)
                    except Exception as e:
                        if os.getenv("TELEROBOT_DEBUG", "0") == "1":
                            log_message(logger, f"Camera read failed for {cam_name}: {type(e).__name__}: {e}")

            t_camera = time.perf_counter()

            loop_count += 1
            maybe_log_loop_timing(logger, loop_count, t0, t_control, t_rerun, t_camera, camera_fps_stats)

            precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - t0), 0.0))

    except KeyboardInterrupt:
        log_message(logger, "\nStopping teleop...")
    finally:
        finalize_dataset(dataset, push_to_hub, logger)


if __name__ == "__main__":
    main()
