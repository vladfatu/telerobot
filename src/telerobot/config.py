"""Load and validate robot configuration from a YAML file."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.configs import Cv2Backends
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower import SOFollower
from lerobot.robots.so_follower.config_so_follower import SOFollowerConfig, SOFollowerRobotConfig
from lerobot.robots.bi_so_follower.config_bi_so_follower import BiSOFollowerConfig
from lerobot.robots.bi_so_follower.bi_so_follower import BiSOFollower


def precision_mode_to_scales(mode: str) -> tuple[float, float]:
    """Convert launcher workspace scale label into position/rotation sensitivity.

    1:1 Normal   -> 1.0
    2:1 Delicate -> 0.5
    5:1 Surgery  -> 0.2
    """
    mode = (mode or "").lower()

    if mode.startswith("5:1") or "surgery" in mode:
        return 0.2, 0.2

    if mode.startswith("2:1") or "delicate" in mode:
        return 0.5, 0.5

    return 1.0, 1.0


@dataclass
class CameraConfig:
    """Configuration for a single camera."""
    index: int | str
    width: int = 640
    height: int = 480
    fps: int = 30
    fourcc: str | None = "MJPG"
    warmup_s: int = 15
    backend: str = "V4L2"


@dataclass
class ArmConfig:
    """Configuration for a single robot arm."""
    type: str
    port: str
    use_degrees: bool = True
    regularization: float = 1e-3
    end_effector_step_sizes: dict[str, float] = field(default_factory=lambda: {"x": 0.5, "y": 0.5, "z": 0.5})
    end_effector_bounds: dict[str, list[float]] = field(
        default_factory=lambda: {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}
    )
    max_ee_step_m: float = 0.20
    gripper_speed_factor: float = 20.0
    cameras: list[str] = field(default_factory=list)


@dataclass
class DatasetConfig:
    """Configuration for recording episodes to a LeRobot dataset."""
    repo_id: str
    single_task: str
    root: str | None = None
    push_to_hub: bool = False


@dataclass
class TeleopConfig:
    """Configuration for VR teleoperation behavior."""
    speed: float = 100.0
    smoothing: float = 5.0
    precision_mode: str = "1:1 (Normal)"
    position_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_trigger_mode: str = "Press Trigger to Close"


@dataclass
class RobotConfig:
    """Top-level robot configuration."""
    id: str
    fps: int
    cameras: dict[str, CameraConfig]
    arms: dict[str, ArmConfig]
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    dataset: DatasetConfig | None = None
    use_rerun: bool = True


def load_config(path: str | Path) -> RobotConfig:
    """Load a robot configuration from a YAML file.

    Args:
        path: Path to the YAML configuration file.

    Returns:
        A validated RobotConfig instance.

    Raises:
        FileNotFoundError: If the config file does not exist.
        ValueError: If required fields are missing.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"Config file not found: {path}\n"
            "Copy config.example.yaml to config.yaml and adjust to match your setup."
        )

    with open(path) as f:
        raw: dict[str, Any] = yaml.safe_load(f)

    # Parse cameras
    cameras: dict[str, CameraConfig] = {}
    for name, cam in raw.get("cameras", {}).items():
        cameras[name] = CameraConfig(
            index=cam["index"],
            width=cam.get("width", 640),
            height=cam.get("height", 480),
            fps=cam.get("fps", 30),
            fourcc=cam.get("fourcc", "MJPG"),
            warmup_s=cam.get("warmup_s", 15),
            backend=cam.get("backend", "V4L2"),
        )

    # Parse arms
    arms: dict[str, ArmConfig] = {}
    for name, arm in raw.get("arms", {}).items():
        arm_cameras = arm.get("cameras", [])
        # Validate that referenced cameras exist
        for cam_name in arm_cameras:
            if cam_name not in cameras:
                raise ValueError(
                    f"Arm '{name}' references camera '{cam_name}' which is not defined in cameras section."
                )
        arms[name] = ArmConfig(
            type=arm["type"],
            port=arm["port"],
            use_degrees=arm.get("use_degrees", True),
            regularization=arm.get("regularization", 1e-3),
            end_effector_step_sizes=arm.get("end_effector_step_sizes", {"x": 0.5, "y": 0.5, "z": 0.5}),
            end_effector_bounds=arm.get("end_effector_bounds", {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}),
            max_ee_step_m=arm.get("max_ee_step_m", 0.20),
            gripper_speed_factor=arm.get("gripper_speed_factor", 20.0),
            cameras=arm_cameras,
        )

    # Parse dataset config (optional)
    dataset_cfg: DatasetConfig | None = None
    dataset_section = raw.get("dataset")
    if dataset_section is not None:
        repo_id = dataset_section.get("repo_id")
        single_task = dataset_section.get("single_task")
        if not repo_id or not single_task:
            raise ValueError("dataset section requires both 'repo_id' and 'single_task'.")
        dataset_cfg = DatasetConfig(
            repo_id=repo_id,
            single_task=single_task,
            root=dataset_section.get("root"),
            push_to_hub=dataset_section.get("push_to_hub", False),
        )

    # Parse teleop config
    teleop_section = raw.get("teleop", {}) or {}
    precision_mode = teleop_section.get("precision_mode", "1:1 (Normal)")
    default_position_scale, default_rotation_scale = precision_mode_to_scales(precision_mode)

    teleop_cfg = TeleopConfig(
        speed=teleop_section.get("speed", 100.0),
        smoothing=teleop_section.get("smoothing", 5.0),
        precision_mode=precision_mode,
        position_scale=teleop_section.get("position_scale", default_position_scale),
        rotation_scale=teleop_section.get("rotation_scale", default_rotation_scale),
        gripper_trigger_mode=teleop_section.get("gripper_trigger_mode", "Press Trigger to Close"),
    )

    robot_section = raw.get("robot", {})
    return RobotConfig(
        id=robot_section.get("id", "duo_robot"),
        fps=robot_section.get("fps", 30),
        cameras=cameras,
        arms=arms,
        teleop=teleop_cfg,
        dataset=dataset_cfg,
        use_rerun=robot_section.get("use_rerun", True),
    )


def load_robot(path: str | Path) -> tuple[Robot, RobotConfig]:
    """Load config and build a ready-to-use robot.

    Returns a BiSOFollower when the config defines two arms ("left" and "right"),
    or a single SOFollower when only one arm is defined.

    Args:
        path: Path to the YAML configuration file.

    Returns:
        A tuple of (robot_instance, config).
    """
    cfg = load_config(path)

    # Build camera configs
    camera_configs = {
        name: OpenCVCameraConfig(
            index_or_path=cam.index,
            width=cam.width,
            height=cam.height,
            fps=cam.fps,
            fourcc=cam.fourcc,
            warmup_s=cam.warmup_s,
            backend=getattr(Cv2Backends, cam.backend),
        )
        for name, cam in cfg.cameras.items()
    }

    # Build arm configs
    arm_configs = {}
    for name, arm in cfg.arms.items():
        arm_cameras = {cam_name: camera_configs[cam_name] for cam_name in arm.cameras}
        arm_configs[name] = SOFollowerConfig(
            port=arm.port,
            use_degrees=arm.use_degrees,
            cameras=arm_cameras if arm_cameras else {},
        )

    if len(arm_configs) == 1:
        # Single-arm configuration
        arm_name, arm_config = next(iter(arm_configs.items()))
        single_config = SOFollowerRobotConfig(
            port=arm_config.port,
            use_degrees=arm_config.use_degrees,
            cameras=arm_config.cameras,
            id=f"{cfg.id}_{arm_name}",
        )
        return SOFollower(single_config), cfg
    else:
        # Dual-arm configuration
        duo_robot_config = BiSOFollowerConfig(
            left_arm_config=arm_configs["left"],
            right_arm_config=arm_configs["right"],
            id=cfg.id,
        )
        return BiSOFollower(duo_robot_config), cfg
