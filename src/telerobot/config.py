"""Load and validate robot configuration from a YAML file."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so_follower.config_so_follower import SOFollowerConfig
from lerobot.robots.bi_so_follower.config_bi_so_follower import BiSOFollowerConfig
from lerobot.robots.bi_so_follower.bi_so_follower import BiSOFollower


@dataclass
class CameraConfig:
    """Configuration for a single camera."""
    index: int
    width: int = 640
    height: int = 480


@dataclass
class ArmConfig:
    """Configuration for a single robot arm."""
    port: str
    use_degrees: bool = True
    cameras: list[str] = field(default_factory=list)


@dataclass
class RobotConfig:
    """Top-level robot configuration."""
    id: str
    fps: int
    cameras: dict[str, CameraConfig]
    arms: dict[str, ArmConfig]


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
            port=arm["port"],
            use_degrees=arm.get("use_degrees", True),
            cameras=arm_cameras,
        )

    robot_section = raw.get("robot", {})
    return RobotConfig(
        id=robot_section.get("id", "duo_robot"),
        fps=robot_section.get("fps", 30),
        cameras=cameras,
        arms=arms,
    )


def load_robot(path: str | Path) -> tuple[BiSOFollower, RobotConfig]:
    """Load config and build a ready-to-use BiSOFollower robot.

    Args:
        path: Path to the YAML configuration file.

    Returns:
        A tuple of (robot_instance, config).
    """
    cfg = load_config(path)

    # Build camera configs
    camera_configs = {
        name: OpenCVCameraConfig(
            index_or_path=cam.index, width=cam.width, height=cam.height, fps=cfg.fps
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

    duo_robot_config = BiSOFollowerConfig(
        left_arm_config=arm_configs["left"],
        right_arm_config=arm_configs["right"],
        id=cfg.id,
    )
    return BiSOFollower(duo_robot_config), cfg
