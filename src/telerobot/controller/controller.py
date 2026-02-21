"""VR teleop controllers for single- and dual-arm follower robots."""

import copy
from abc import ABC, abstractmethod

from lerobot.processor import RobotObservation
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.robots.bi_so_follower.bi_so_follower import BiSOFollower

from telerobot.config import ArmConfig, RobotConfig
from telerobot.controller.vr_processor import build_vr_to_arm_processor
from telerobot.controller.kinematics import build_kinematics


class Controller(ABC):
    """Base class for VR teleop controllers that manage processors and arm dispatch."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        self.robot = robot
        self.cfg = cfg
        self.has_initial_position = True

    def _build_processor(self, motor_names: list[str], arm_cfg: ArmConfig):
        """Create a kinematics solver and VR-to-arm processor pipeline."""
        kinematics_solver = build_kinematics(
            arm_type=arm_cfg.type,
            motor_names=motor_names,
            regularization=arm_cfg.regularization,
        )
        return build_vr_to_arm_processor(
            motor_names=motor_names,
            kinematics_solver=kinematics_solver,
            end_effector_step_sizes=arm_cfg.end_effector_step_sizes,
            end_effector_bounds=arm_cfg.end_effector_bounds,
            max_ee_step_m=arm_cfg.max_ee_step_m,
            gripper_speed_factor=arm_cfg.gripper_speed_factor,
        )

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


class SingleController(Controller):
    """Controller for a single SOFollower arm."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self.arm_name = next(iter(cfg.arms))  # "left" or "right" — matches VR controller side
        self._build_processors()

    def _build_processors(self) -> None:
        arm_cfg = self.cfg.arms[self.arm_name]
        self.processor = self._build_processor(
            list(self.robot.bus.motors.keys()),
            arm_cfg=arm_cfg,
        )

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


class BiController(Controller):
    """Controller for a BiSOFollower (dual-arm) robot."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self._build_processors()

    def _build_processors(self) -> None:
        self.processors = {
            "left": self._build_processor(
                list(self.robot.left_arm.bus.motors.keys()),
                arm_cfg=self.cfg.arms["left"],
            ),
            "right": self._build_processor(
                list(self.robot.right_arm.bus.motors.keys()),
                arm_cfg=self.cfg.arms["right"],
            ),
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


def build_controller(robot: Robot, cfg: RobotConfig) -> Controller:
    """Create the appropriate controller based on the robot type."""
    if isinstance(robot, SOFollower):
        return SingleController(robot, cfg)
    if isinstance(robot, BiSOFollower):
        return BiController(robot, cfg)
    raise TypeError(f"Unsupported robot type: {type(robot).__name__}")
