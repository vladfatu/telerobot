import argparse
import json
import time
from pathlib import Path

from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data  # noqa: F401 (imported conditionally)

from telerobot.config import load_robot
from telerobot.controller import build_controller
from telerobot.dataset import setup_dataset, end_active_episode, record_step, finalize_dataset, delete_episodes_from_dataset
from telerobot.logger import get_logger, log_message, maybe_log_loop_timing
from telerobot.server import setup_webxr_server, setup_websocket_server

# Resolve default config path relative to the project root (two levels up from this file)
DEFAULT_CONFIG_PATH = str(Path(__file__).parent.parent.parent / "config.yaml")


def main():
    logger = get_logger()

    parser = argparse.ArgumentParser(description="Telerobot — VR teleoperation for SO-ARM101")
    subparsers = parser.add_subparsers(dest="command")

    # --- run (default) ---
    run_parser = subparsers.add_parser("run", help="Start the VR teleoperation loop")
    run_parser.add_argument(
        "-c", "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )

    # --- delete-episodes ---
    del_parser = subparsers.add_parser("delete-episodes", help="Delete episodes from a dataset")
    del_parser.add_argument(
        "--repo-id", required=True,
        help="Repository ID of the dataset (e.g. 'user/dataset-name')",
    )
    del_parser.add_argument(
        "--episodes", required=True,
        help='Episode indices to delete as a JSON list, e.g. "[0, 2, 5]"',
    )
    del_parser.add_argument(
        "--root", default=None,
        help="Optional root directory override for the dataset",
    )
    del_parser.add_argument(
        "--push-to-hub", action="store_true", default=False,
        help="Push the updated dataset to the Hugging Face Hub after deletion",
    )

    args = parser.parse_args()

    # Default to "run" when no subcommand is given (backward-compatible)
    if args.command is None:
        args = run_parser.parse_args()
        args.command = "run"

    if args.command == "delete-episodes":
        episode_indices = json.loads(args.episodes)
        if not isinstance(episode_indices, list) or not all(isinstance(i, int) for i in episode_indices):
            parser.error("--episodes must be a JSON list of integers, e.g. '[0, 2, 5]'")
        delete_episodes_from_dataset(
            repo_id=args.repo_id,
            episode_indices=episode_indices,
            root=args.root,
            push_to_hub=args.push_to_hub,
            logger=logger,
        )
        return

    # --- run command ---

    config_path = getattr(args, "config", DEFAULT_CONFIG_PATH)
    duo_robot, cfg = load_robot(config_path)

    camera_server = setup_webxr_server(duo_robot, logger, dataset_configured=cfg.dataset is not None)
    teleop_device = setup_websocket_server()

    controller = build_controller(duo_robot, cfg)
    dataset = setup_dataset(duo_robot, cfg, logger)

    # Connect to the robot
    duo_robot.connect()

    # Init rerun viewer (optional)
    if cfg.use_rerun:
        init_rerun(session_name="vr_lerobot_teleop")

    if not duo_robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    controller.capture_initial_observations()

    recording = False
    finalized_dataset = False
    push_to_hub = cfg.dataset.push_to_hub if cfg.dataset else False

    log_message(logger, "Starting teleop loop. Connect your VR headset to teleoperate the robot...")
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
            else:
                action_str = vr_obs.get('action', 'none')

                if action_str == 'reset' and not controller.has_initial_position:
                    controller.reset()
                elif action_str == 'start_episode' and not recording:
                    # Begin a new recording episode (no-op if already recording)
                    log_message(logger, f"🔴 Recording episode {dataset.num_episodes if dataset else '?'}...")
                    recording = True
                    finalized_dataset = False
                elif action_str == 'stop_episode' and recording:
                    controller.reset()
                    # End the current recording episode
                    end_active_episode(dataset, logger)
                    recording = False
                elif action_str == 'save_dataset' and not finalized_dataset:
                    # Finalize and save the entire dataset
                    finalize_dataset(dataset, push_to_hub, logger)
                    recording = False
                    finalized_dataset = True
                else:
                    result = controller.process_vr_observation(vr_obs)
                    t_control = time.perf_counter()  # TODO: Remove timing debug

                    if result is not None:
                        obs, action = result

                        # Collect camera frames from observation (avoids double camera read)
                        for cam_name in duo_robot.cameras:
                            if cam_name in obs:
                                camera_frames[cam_name] = obs[cam_name]

                        if cfg.use_rerun:
                            log_rerun_data(observation=obs, action=action)

                        t_rerun = time.perf_counter()  # TODO: Remove timing debug

                        if recording:
                            record_step(dataset, cfg, obs, action)

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
        finalize_dataset(dataset, push_to_hub, logger)


if __name__ == "__main__":
    main()
