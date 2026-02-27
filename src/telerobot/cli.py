import argparse
import time

from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from telerobot.config import load_robot
from telerobot.controller import build_controller
from telerobot.dataset import setup_dataset, end_active_episode, record_step, finalize_dataset
from telerobot.logger import get_logger, log_message, maybe_log_loop_timing
from telerobot.server import setup_camera_server, setup_websocket_teleop

DEFAULT_CONFIG_PATH = "config.yaml"


def main():
    logger = get_logger()

    parser = argparse.ArgumentParser(description="Telerobot — VR teleoperation for SO-ARM101")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )
    args = parser.parse_args()

    duo_robot, cfg = load_robot(args.config)

    camera_server = setup_camera_server(duo_robot, logger)
    teleop_device = setup_websocket_teleop()

    controller = build_controller(duo_robot, cfg)
    dataset = setup_dataset(duo_robot, cfg, logger)

    # Connect to the robot
    duo_robot.connect()

    # Init rerun viewer
    init_rerun(session_name="vr_lerobot_teleop")

    if not duo_robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    controller.capture_initial_observations()

    recording = False

    log_message(logger, "Starting teleop loop. Move your phone to teleoperate the robot...")
    loop_count = 0

    try:
        while True:
            t0 = time.perf_counter()
            t_control = t_rerun = t_dataset = None  # TODO: Remove timing debug

            # Get teleop action
            vr_obs = teleop_device.last_observation
            camera_frames = {}  # Populated by process_vr_observation if available

            if vr_obs is None:
                pass  # No observation received yet; cameras still streamed below
            elif vr_obs['reset'] and not controller.has_initial_position:
                controller.reset()
                # End current episode if we were recording
                recording = end_active_episode(dataset, recording, logger)
            else:
                # print("VR Observation: ", vr_obs)
                result = controller.process_vr_observation(vr_obs)

                t_control = time.perf_counter()  # TODO: Remove timing debug

                if result is not None:
                    obs, action = result

                    # Collect camera frames from observation (avoids double camera read)
                    for cam_name in duo_robot.cameras:
                        if cam_name in obs:
                            camera_frames[cam_name] = obs[cam_name]

                    log_rerun_data(observation=obs, action=action)

                    t_rerun = time.perf_counter()  # TODO: Remove timing debug

                    recording = record_step(dataset, cfg, obs, action, recording, logger)

            # Always stream cameras — reuse obs frames when available, otherwise read directly
            for cam_name, cam in duo_robot.cameras.items():
                frame = camera_frames.get(cam_name) if camera_frames else cam.async_read()
                camera_server.update_camera_frame(cam_name, frame)

            t_camera = time.perf_counter()  # TODO: Remove timing debug

            loop_count += 1
            maybe_log_loop_timing(logger, loop_count, t0, t_control, t_rerun, t_camera)

            precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - t0), 0.0))

    except KeyboardInterrupt:
        log_message(logger, "\nStopping teleop...")
    finally:
        finalize_dataset(dataset, recording, logger)


if __name__ == "__main__":
    main()
