import argparse
import time
import threading

from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, ACTION, OBS_STR
from lerobot.processor import make_default_processors
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


def setup_dataset(robot, cfg) -> LeRobotDataset | None:
    """Create a LeRobotDataset for episode recording, or None if not configured."""
    if cfg.dataset is None:
        return None

    teleop_action_processor, _robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    num_cameras = len(robot.cameras) if hasattr(robot, "cameras") else 0
    dataset = LeRobotDataset.create(
        repo_id=cfg.dataset.repo_id,
        fps=cfg.fps,
        root=cfg.dataset.root,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * max(num_cameras, 1),
        vcodec=cfg.dataset.vcodec,
    )

    print(f"📁 Dataset recording enabled: {cfg.dataset.repo_id}")
    return dataset


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
    dataset = setup_dataset(duo_robot, cfg)

    # Connect to the robot
    duo_robot.connect()

    # Init rerun viewer
    init_rerun(session_name="vr_lerobot_teleop")

    if not duo_robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    controller.capture_initial_observations()

    recording = False

    print("Starting teleop loop. Move your phone to teleoperate the robot...")
    # TODO: Remove timing debug prints
    loop_count = 0
    timing_interval = 100  # Print every N loops

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
                if recording and dataset is not None:
                    dataset.save_episode()
                    print(f"✅ Episode {dataset.num_episodes - 1} saved "
                          f"({dataset.num_frames} total frames)")
                    recording = False
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

                    if dataset is not None:
                        observation_frame = build_dataset_frame(
                            dataset.features, obs, prefix=OBS_STR
                        )
                        action_frame = build_dataset_frame(
                            dataset.features, action, prefix=ACTION
                        )
                        frame = {
                            **observation_frame,
                            **action_frame,
                            "task": cfg.dataset.single_task,
                        }
                        dataset.add_frame(frame)

                        if not recording:
                            print(f"🔴 Recording episode {dataset.num_episodes}...")
                            recording = True

                    t_dataset = time.perf_counter()  # TODO: Remove timing debug

            # Always stream cameras — reuse obs frames when available, otherwise read directly
            for cam_name, cam in duo_robot.cameras.items():
                frame = camera_frames.get(cam_name) if camera_frames else cam.async_read()
                camera_server.update_camera_frame(cam_name, frame)

            t_camera = time.perf_counter()  # TODO: Remove timing debug

            # TODO: Remove timing debug prints
            loop_count += 1
            if loop_count % timing_interval == 0:
                total = time.perf_counter() - t0
                parts = []
                if t_control is not None:
                    parts.append(f"control+IK: {(t_control - t0)*1000:.1f}ms")
                if t_rerun is not None and t_control is not None:
                    parts.append(f"rerun: {(t_rerun - t_control)*1000:.1f}ms")
                if t_dataset is not None and t_rerun is not None:
                    parts.append(f"dataset: {(t_dataset - t_rerun)*1000:.1f}ms")
                parts.append(f"camera stream: {(t_camera - (t_dataset or t_rerun or t_control or t0))*1000:.1f}ms")
                parts.append(f"total: {total*1000:.1f}ms")
                parts.append(f"effective fps: {1/total:.1f}")
                print(f"⏱ Loop {loop_count} | " + " | ".join(parts))

            precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - t0), 0.0))

    except KeyboardInterrupt:
        print("\nStopping teleop...")
    finally:
        if dataset is not None:
            if recording:
                dataset.save_episode()
                print(f"✅ Episode {dataset.num_episodes - 1} saved "
                      f"({dataset.num_frames} total frames)")
            safe_stop_image_writer(dataset)
            print(f"📊 Total episodes recorded: {dataset.num_episodes}")


if __name__ == "__main__":
    main()
