import argparse
import time
import threading

from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from telerobot.config import load_robot
from telerobot.controller import build_controller
from telerobot.server import VRHeadset, create_camera_server

DEFAULT_CONFIG_PATH = "config.yaml"


def setup_camera_server(robot) -> None:
    """Initialize and start the WebRTC camera server in a background thread."""
    use_https = True  # Set to False for HTTP
    cert_file = "ssl_cert/server.crt"
    key_file = "ssl_cert/server.key"

    camera_server = create_camera_server(
        robot.cameras.keys(),
        use_https=use_https,
        cert_file=cert_file,
        key_file=key_file
    )

    server_thread = threading.Thread(target=camera_server.run_in_thread, daemon=True)
    server_thread.start()

    if use_https:
        print("🔒 HTTPS WebRTC camera server started on https://0.0.0.0:8765")
        print("📱 Access from Quest 3: https://YOUR_IP:8765")
    else:
        print("🎥 HTTP WebRTC camera server started on http://0.0.0.0:8765")

    return camera_server


def setup_websocket_teleop() -> VRHeadset:
    """Create and connect the VR headset WebSocket teleop device."""
    teleop_device = VRHeadset()
    teleop_device.connect()
    return teleop_device


def main():
    parser = argparse.ArgumentParser(description="Telerobot — VR teleoperation for SO-ARM101")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )
    args = parser.parse_args()

    duo_robot, cfg = load_robot(args.config)

    camera_server = setup_camera_server(duo_robot)
    teleop_device = setup_websocket_teleop()

    controller = build_controller(duo_robot, cfg)

    # Connect to the robot
    duo_robot.connect()

    # Init rerun viewer
    init_rerun(session_name="vr_lerobot_teleop")

    if not duo_robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    controller.capture_initial_observations()

    print("Starting teleop loop. Move your phone to teleoperate the robot...")
    while True:
        t0 = time.perf_counter()

        # Capture and stream camera frames
        try:
            for cam_name, cam in duo_robot.cameras.items():
                frame = cam.async_read(timeout_ms=50)
                if frame is not None:
                    camera_server.update_camera_frame(cam_name, frame)
        except Exception as e:
            print(f"Error capturing camera frames: {e}")

        # Get teleop action
        vr_obs = teleop_device.last_observation

        if vr_obs is None:
            log_rerun_data(observation=duo_robot.get_observation(), action=None)
        elif vr_obs['reset'] and not controller.has_initial_position:
            controller.reset()
        else:
            print("VR Observation: ", vr_obs)
            controller.process_vr_observation(vr_obs)

        # busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

        ## TODO remove - for testing only
        precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()
