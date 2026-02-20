import argparse
import time
import cv2
import numpy as np
import copy
import threading

from abc import ABC, abstractmethod

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.robots.bi_so_follower.bi_so_follower import BiSOFollower
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from telerobot import PACKAGE_DIR
from telerobot.config import RobotConfig, load_robot
from telerobot.server import VRHeadset, create_camera_server
from telerobot.vr_processor import MapVRActionToRobotAction

DEFAULT_CONFIG_PATH = "config.yaml"


# NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo:
# https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
def get_kinematics_solver(motor_names: list[str]) -> RobotKinematics:
    kin = RobotKinematics(
        urdf_path=str(PACKAGE_DIR / "robots" / "SO101" / "so101_new_calib.urdf"),
        target_frame_name="gripper_frame_link",
        joint_names=motor_names,
    )
    # Regularization prevents singularity-induced oscillation at full extension.
    # The L2 penalty on ||dq||² keeps the QP well-conditioned so the solver
    # returns a stable, unique solution near workspace boundaries.
    kin.solver.add_regularization_task(1e-3)
    return kin


# Build pipeline to convert phone action to ee pose action to joint action
def get_vr_to_arm_processor(motor_names: list[str]) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    kinematics_solver = get_kinematics_solver(motor_names)
    return RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[
            MapVRActionToRobotAction(),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.20,
            ),
            GripperVelocityToJoint(
                speed_factor=20.0,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )


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


class FollowerController(ABC):
    """Base class for VR teleop controllers that manage processors and arm dispatch."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        self.robot = robot
        self.cfg = cfg
        self.has_initial_position = True

    @abstractmethod
    def _build_processors(self) -> None:
        """Create VR-to-arm processor pipelines."""

    @abstractmethod
    def capture_initial_observations(self) -> None:
        """Capture the initial arm observations used for reset."""

    @abstractmethod
    def reset(self) -> None:
        """Reset the robot to its initial position and rebuild processors."""

    @abstractmethod
    def get_arm_observations(self) -> dict[str, RobotObservation]:
        """Return per-arm observations keyed by arm name."""

    @abstractmethod
    def process_vr_observation(self, vr_obs: dict) -> None:
        """Dispatch a VR observation to the appropriate arms."""


class SingleFollowerController(FollowerController):
    """Controller for a single SOFollower arm."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self.arm_name = next(iter(cfg.arms))  # "left" or "right" — matches VR controller side
        self._build_processors()

    def _build_processors(self) -> None:
        self.processor = get_vr_to_arm_processor(list(self.robot.bus.motors.keys()))

    def capture_initial_observations(self) -> None:
        self.initial_obs = self.robot.get_observation()

    def reset(self) -> None:
        print("Resetting robot to initial position...")
        self.robot.send_action(self.initial_obs)
        self._build_processors()
        self.has_initial_position = True

    def get_arm_observations(self) -> dict[str, RobotObservation]:
        return {self.arm_name: self.robot.get_observation()}

    def process_vr_observation(self, vr_obs: dict) -> None:
        controller_obs = copy.deepcopy(vr_obs[self.arm_name])

        if controller_obs["enabled"]:
            self.has_initial_position = False
            print(f"{self.arm_name.capitalize()} Arm VR Position: {controller_obs['pos']}")
        else:
            print(f"{self.arm_name.capitalize()} controller not enabled.")

        obs = self.robot.get_observation()
        joint_action = self.processor((controller_obs, obs))
        self.robot.send_action(joint_action)


class BiFollowerController(FollowerController):
    """Controller for a BiSOFollower (dual-arm) robot."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self._build_processors()

    def _build_processors(self) -> None:
        self.processors = {
            "left": get_vr_to_arm_processor(list(self.robot.left_arm.bus.motors.keys())),
            "right": get_vr_to_arm_processor(list(self.robot.right_arm.bus.motors.keys())),
        }

    def capture_initial_observations(self) -> None:
        self.initial_left_obs = self.robot.left_arm.get_observation()
        self.initial_right_obs = self.robot.right_arm.get_observation()

    def reset(self) -> None:
        print("Resetting robot to initial position...")
        self.robot.right_arm.send_action(self.initial_right_obs)
        self.robot.left_arm.send_action(self.initial_left_obs)
        self._build_processors()
        self.has_initial_position = True

    def get_arm_observations(self) -> dict[str, RobotObservation]:
        return {
            "left": self.robot.left_arm.get_observation(),
            "right": self.robot.right_arm.get_observation(),
        }

    def process_vr_observation(self, vr_obs: dict) -> None:
        for side in ("right", "left"):
            arm = getattr(self.robot, f"{side}_arm")
            controller_obs = copy.deepcopy(vr_obs[side])

            if controller_obs["enabled"]:
                self.has_initial_position = False
                print(f"{side.capitalize()} Arm VR Position: {controller_obs['pos']}")
            else:
                print(f"{side.capitalize()} controller not enabled.")

            obs = arm.get_observation()
            joint_action = self.processors[side]((controller_obs, obs))
            arm.send_action(joint_action)


def build_controller(robot: Robot, cfg: RobotConfig) -> FollowerController:
    """Create the appropriate controller based on the robot type."""
    if isinstance(robot, SOFollower):
        return SingleFollowerController(robot, cfg)
    if isinstance(robot, BiSOFollower):
        return BiFollowerController(robot, cfg)
    raise TypeError(f"Unsupported robot type: {type(robot).__name__}")


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
