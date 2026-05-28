"""VR teleop controllers for unified modular robot configurations."""

import copy
import time
from abc import ABC, abstractmethod

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot  # Unified Robot interface in v0.5+

from telerobot.config import ArmConfig, RobotConfig
from telerobot.controller.vr_processor import MapVRActionToRobotAction, build_vr_to_arm_processor
from telerobot.controller.kinematics import build_kinematics


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _numeric_keys(obs: dict) -> list[str]:
    """Return keys that look like scalar motor/joint values."""
    return [k for k, v in obs.items() if _is_number(v)]


def _distance(a: dict, b: dict, keys: list[str]) -> float:
    """Mean absolute distance over numeric keys."""
    if not keys:
        return 0.0
    return sum(abs(float(a.get(k, 0.0)) - float(b.get(k, 0.0))) for k in keys) / len(keys)


def _interpolate_action(start: dict, target: dict, keys: list[str], alpha: float) -> dict:
    """Build a small interpolated action from start observation to target observation."""
    action = {}
    for k in keys:
        sv = float(start[k])
        tv = float(target[k])
        action[k] = sv + (tv - sv) * alpha
    return action


class Controller(ABC):
    """Base class for VR teleop controllers that manage processors and arm dispatch."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        self.robot = robot
        self.cfg = cfg
        self.has_initial_position = True
        self._base_control_debug_printed = False
        self._base_button_targets = {}
        self._base_button_last_print = 0.0

        # Latest live runtime settings. Processor rebuilds after reset must use
        # these values, not the original config.yaml values.
        self._runtime_position_scale = getattr(self.cfg.teleop, "position_scale", 1.0)
        self._runtime_rotation_scale = getattr(self.cfg.teleop, "rotation_scale", 1.0)
        self._runtime_gripper_trigger_mode = getattr(
            self.cfg.teleop,
            "gripper_trigger_mode",
            "Press Trigger to Close",
        )

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
            position_scale=self._runtime_position_scale,
            rotation_scale=self._runtime_rotation_scale,
            gripper_trigger_mode=self._runtime_gripper_trigger_mode,
        )

    def _iter_vr_processors(self):
        """Yield active VR processor pipelines without rebuilding them."""
        if hasattr(self, "processor"):
            yield self.processor

        if hasattr(self, "processors"):
            for processor in self.processors.values():
                yield processor

    def update_runtime_settings(
        self,
        position_scale: float | None = None,
        rotation_scale: float | None = None,
        gripper_trigger_mode: str | None = None,
    ) -> bool:
        """Apply and remember live teleop settings.

        Remembered values are used when reset/soft_reset rebuilds the processor
        pipeline, so runtime settings survive reset.
        """
        if position_scale is not None:
            self._runtime_position_scale = float(position_scale)

        if rotation_scale is not None:
            self._runtime_rotation_scale = float(rotation_scale)

        if gripper_trigger_mode is not None:
            if gripper_trigger_mode not in ("Press Trigger to Close", "Press Trigger to Open"):
                gripper_trigger_mode = "Press Trigger to Close"
            self._runtime_gripper_trigger_mode = gripper_trigger_mode

        any_changed = False

        for processor in self._iter_vr_processors():
            for step in getattr(processor, "steps", []):
                if isinstance(step, MapVRActionToRobotAction):
                    changed = step.update_runtime_settings(
                        position_scale=self._runtime_position_scale,
                        rotation_scale=self._runtime_rotation_scale,
                        gripper_trigger_mode=self._runtime_gripper_trigger_mode,
                    )
                    any_changed = any_changed or changed

        return any_changed

    def _find_base_joint_key(self, joint_action: RobotAction, obs: RobotObservation) -> str | None:
        """Find the most likely base/shoulder pan joint key."""
        candidate_keys = list(joint_action.keys())

        # Prefer common names first.
        preferred_fragments = [
            "shoulder_pan",
            "base",
            "waist",
            "pan",
        ]

        for fragment in preferred_fragments:
            for key in candidate_keys:
                key_lower = str(key).lower()
                if fragment in key_lower and "gripper" not in key_lower:
                    return key

        # If no obvious key is found, return None.
        # We do NOT blindly use the first joint because that could move
        # the wrong motor.
        return None

    def _apply_base_button_control(
        self,
        joint_action: RobotAction,
        obs: RobotObservation,
        controller_obs: dict,
    ) -> RobotAction:
        """Temporarily disabled A/B base rotation for safety.

        The experimental base-rotation version could fight the IK/sculpting
        pipeline and cause unsafe snap-back behavior after releasing A/B.

        Keep this as a no-op until base rotation is redesigned as a proper
        latched joint-space mode.
        """
        return joint_action

        # When no base button is pressed, keep the persistent target synced
        # to the latest observed/action value and do not override IK.
        if button_a == button_b:
            try:
                self._base_button_targets[base_key] = float(obs.get(base_key, joint_action.get(base_key, 0.0)))
            except Exception:
                pass
            return joint_action

        # Determine whether robot action units are likely degrees or radians.
        use_degrees = True
        try:
            use_degrees = any(arm.use_degrees for arm in self.cfg.arms.values())
        except Exception:
            use_degrees = True

        # Conservative per-loop step.
        # At robot.fps=10, 1.0 degree/loop is about 10 deg/s.
        # If this feels too fast, reduce to 0.5.
        step = 1.0 if use_degrees else 0.01745

        direction = 0.0
        if button_a:
            direction += 1.0
        if button_b:
            direction -= 1.0

        # Initialize persistent target from current observation/action.
        if base_key not in self._base_button_targets:
            try:
                self._base_button_targets[base_key] = float(obs.get(base_key, joint_action.get(base_key, 0.0)))
            except Exception:
                self._base_button_targets[base_key] = float(joint_action.get(base_key, 0.0))

        self._base_button_targets[base_key] += direction * step
        joint_action[base_key] = self._base_button_targets[base_key]

        # Compact debug print, rate-limited.
        now = time.time()
        if now - getattr(self, "_base_button_last_print", 0.0) > 0.5:
            print(
                f"BASE_CONTROL_APPLY: key={base_key}, "
                f"A={button_a}, B={button_b}, "
                f"target={joint_action[base_key]:.3f}, step={step}"
            )
            self._base_button_last_print = now

        return joint_action

        base_key = self._find_base_joint_key(joint_action, obs)

        if not self._base_control_debug_printed:
            print("BASE_CONTROL_DEBUG:")
            print(f"  buttonA={button_a}, buttonB={button_b}")
            print(f"  chosen_base_key={base_key}")
            print(f"  joint_action_keys={list(joint_action.keys())}")
            print(f"  observation_keys={list(obs.keys())}")
            try:
                print(f"  robot_motor_names={list(self.robot.motor_names)}")
            except Exception:
                pass
            self._base_control_debug_printed = True

        if base_key is None:
            print("BASE_CONTROL_DEBUG: No base joint key found. Base rotation skipped.")
            return joint_action

        # Determine whether robot action units are likely degrees or radians.
        use_degrees = True
        try:
            use_degrees = any(arm.use_degrees for arm in self.cfg.arms.values())
        except Exception:
            use_degrees = True

        # Conservative per-loop step.
        # At robot.fps=10, 0.5 degrees/loop is about 5 deg/s.
        step = 0.5 if use_degrees else 0.0087

        direction = 0.0
        if button_a:
            direction += 1.0
        if button_b:
            direction -= 1.0

        current_value = obs.get(base_key, joint_action.get(base_key, 0.0))

        try:
            joint_action[base_key] = float(current_value) + direction * step
        except Exception as e:
            print(f"BASE_CONTROL_DEBUG: Failed to apply base rotation on {base_key}: {e}")

        return joint_action

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
    def process_vr_observation(self, vr_obs: dict) -> tuple[RobotObservation, RobotAction] | None:
        """Dispatch a VR observation to the appropriate arms.

        Returns:
            A (observation, action) tuple for dataset recording, or None.
        """


class SingleController(Controller):
    """Controller for a single-arm modular robot."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self.arm_name = next(iter(cfg.arms))  # "left" or "right" — matches VR controller side
        self._build_processors()

    def _build_processors(self) -> None:
        arm_cfg = self.cfg.arms[self.arm_name]
        # Handle unified motor naming in v0.5+
        motor_names = list(self.robot.motor_names) if hasattr(self.robot, "motor_names") else list(self.robot.bus.motors.keys())
        self.processor = self._build_processor(
            motor_names,
            arm_cfg=arm_cfg,
        )

    def capture_initial_observations(self) -> None:
        self.initial_obs = self.robot.get_observation()

    def soft_reset(
        self,
        duration_s: float = 5.0,
        steps: int = 60,
        stall_window: int = 8,
        min_progress: float = 0.0005,
        remaining_error_threshold: float = 0.01,
    ) -> None:
        """Slowly return the robot to the captured initial pose.

        This avoids the harsh one-shot reset. If the arm appears stalled
        while still far from the target, the reset stops early.
        """
        print("Soft resetting robot to initial position...")

        if not hasattr(self, "initial_obs"):
            print("Soft reset skipped: no initial observation captured.")
            return

        try:
            start_obs = self.robot.get_observation()
        except Exception as e:
            print(f"Soft reset skipped: could not read current observation: {e}")
            return

        target_obs = self.initial_obs
        keys = [k for k in _numeric_keys(target_obs) if k in start_obs and _is_number(start_obs[k])]

        if not keys:
            print("Soft reset skipped: no numeric motor keys found.")
            return

        initial_error = _distance(start_obs, target_obs, keys)
        if initial_error < remaining_error_threshold:
            print("Soft reset skipped: robot is already near initial pose.")
            self._build_processors()
            self.has_initial_position = True
            return

        sleep_s = max(duration_s / max(steps, 1), 0.02)
        last_obs = start_obs
        low_progress_count = 0

        for i in range(1, steps + 1):
            alpha = i / steps
            action = _interpolate_action(start_obs, target_obs, keys, alpha)

            self.robot.send_action(action)
            time.sleep(sleep_s)

            try:
                current_obs = self.robot.get_observation()
            except Exception as e:
                print(f"Soft reset stopped: could not read observation: {e}")
                break

            remaining_error = _distance(current_obs, target_obs, keys)
            progress = _distance(current_obs, last_obs, keys)

            # If we are still far from target but the arm barely moves for
            # several cycles, assume something may be resisting/blocking motion.
            if remaining_error > remaining_error_threshold and progress < min_progress:
                low_progress_count += 1
            else:
                low_progress_count = 0

            if low_progress_count >= stall_window:
                print("Soft reset stopped early: movement appears stalled or resisted.")
                break

            if remaining_error <= remaining_error_threshold:
                print("Soft reset complete.")
                break

            last_obs = current_obs

        self._build_processors()
        self.has_initial_position = True

    def reset(self) -> None:
        print("Resetting robot to initial position...")
        self.robot.send_action(self.initial_obs)
        self._build_processors()
        self.has_initial_position = True

    def get_arm_observations(self) -> dict[str, RobotObservation]:
        return {self.arm_name: self.robot.get_observation()}

    def process_vr_observation(self, vr_obs: dict) -> tuple[RobotObservation, RobotAction] | None:
        controller_obs = copy.deepcopy(vr_obs[self.arm_name])

        if controller_obs["enabled"]:
            self.has_initial_position = False

        obs = self.robot.get_observation()
        joint_action = self.processor((controller_obs, obs))
        joint_action = self._apply_base_button_control(joint_action, obs, controller_obs)
        self.robot.send_action(joint_action)
        return obs, joint_action


class BiController(Controller):
    """Controller for a dual-arm modular robot."""

    def __init__(self, robot: Robot, cfg: RobotConfig):
        super().__init__(robot, cfg)
        self._build_processors()

    def _get_arm_motors(self, side: str) -> list[str]:
        """Extract motor names for a specific arm from the namespace."""
        if hasattr(self.robot, "motor_names"):
            # V0.5+ Unified namespace (e.g., 'left_shoulder_pan')
            return [m for m in self.robot.motor_names if m.startswith(side)]
        else:
            # Fallback if the robot config exposes arms as legacy sub-objects
            arm = getattr(self.robot, f"{side}_arm")
            return list(arm.bus.motors.keys())

    def _build_processors(self) -> None:
        self.processors = {
            "left": self._build_processor(
                self._get_arm_motors("left"),
                arm_cfg=self.cfg.arms["left"],
            ),
            "right": self._build_processor(
                self._get_arm_motors("right"),
                arm_cfg=self.cfg.arms["right"],
            ),
        }

    def capture_initial_observations(self) -> None:
        if hasattr(self.robot, "get_observation"):
            # Unified observation dictionary in v0.5+
            self.initial_obs = self.robot.get_observation()
        else:
            self.initial_left_obs = self.robot.left_arm.get_observation()
            self.initial_right_obs = self.robot.right_arm.get_observation()

    def soft_reset(
        self,
        duration_s: float = 5.0,
        steps: int = 60,
        stall_window: int = 8,
        min_progress: float = 0.0005,
        remaining_error_threshold: float = 0.01,
    ) -> None:
        """Slowly return dual-arm/unified robot to captured initial pose."""
        print("Soft resetting robot to initial position...")

        if not hasattr(self, "initial_obs"):
            # Legacy dual-arm fallback: keep old behavior for now.
            print("Soft reset fallback: no unified initial observation found.")
            self.reset()
            return

        try:
            start_obs = self.robot.get_observation()
        except Exception as e:
            print(f"Soft reset skipped: could not read current observation: {e}")
            return

        target_obs = self.initial_obs
        keys = [k for k in _numeric_keys(target_obs) if k in start_obs and _is_number(start_obs[k])]

        if not keys:
            print("Soft reset skipped: no numeric motor keys found.")
            return

        initial_error = _distance(start_obs, target_obs, keys)
        if initial_error < remaining_error_threshold:
            print("Soft reset skipped: robot is already near initial pose.")
            self._build_processors()
            self.has_initial_position = True
            return

        sleep_s = max(duration_s / max(steps, 1), 0.02)
        last_obs = start_obs
        low_progress_count = 0

        for i in range(1, steps + 1):
            alpha = i / steps
            action = _interpolate_action(start_obs, target_obs, keys, alpha)

            self.robot.send_action(action)
            time.sleep(sleep_s)

            try:
                current_obs = self.robot.get_observation()
            except Exception as e:
                print(f"Soft reset stopped: could not read observation: {e}")
                break

            remaining_error = _distance(current_obs, target_obs, keys)
            progress = _distance(current_obs, last_obs, keys)

            if remaining_error > remaining_error_threshold and progress < min_progress:
                low_progress_count += 1
            else:
                low_progress_count = 0

            if low_progress_count >= stall_window:
                print("Soft reset stopped early: movement appears stalled or resisted.")
                break

            if remaining_error <= remaining_error_threshold:
                print("Soft reset complete.")
                break

            last_obs = current_obs

        self._build_processors()
        self.has_initial_position = True

    def reset(self) -> None:
        print("Resetting robot to initial position...")
        if hasattr(self, "initial_obs"):
            self.robot.send_action(self.initial_obs)
        else:
            self.robot.right_arm.send_action(self.initial_right_obs)
            self.robot.left_arm.send_action(self.initial_left_obs)
        self._build_processors()
        self.has_initial_position = True

    def get_arm_observations(self) -> dict[str, RobotObservation]:
        if hasattr(self.robot, "get_observation"):
            full_obs = self.robot.get_observation()
            return {"left": full_obs, "right": full_obs}
        return {
            "left": self.robot.left_arm.get_observation(),
            "right": self.robot.right_arm.get_observation(),
        }

    def process_vr_observation(self, vr_obs: dict) -> tuple[RobotObservation, RobotAction] | None:
        combined_obs: RobotObservation = {}
        combined_action: RobotAction = {}

        # Handle both unified robot and legacy sub-arm dispatch seamlessly
        is_unified = hasattr(self.robot, "get_observation")

        if is_unified:
            full_obs = self.robot.get_observation()
            combined_obs.update(full_obs)

        for side in ("right", "left"):
            controller_obs = copy.deepcopy(vr_obs[side])

            if controller_obs["enabled"]:
                self.has_initial_position = False
                print(f"{side.capitalize()} Arm VR Position: {controller_obs['pos']}")
            else:
                print(f"{side.capitalize()} controller not enabled.")

            if is_unified:
                obs = full_obs
            else:
                arm = getattr(self.robot, f"{side}_arm")
                obs = arm.get_observation()
                combined_obs.update(obs)

            joint_action = self.processors[side]((controller_obs, obs))
            joint_action = self._apply_base_button_control(joint_action, obs, controller_obs)

            if not is_unified:
                arm.send_action(joint_action)

            combined_action.update(joint_action)

        if is_unified:
            self.robot.send_action(combined_action)

        # Merge any full-system observations (e.g. cameras) missing from combined_obs
        if not is_unified:
            full_obs = self.robot.get_observation()
            for key in full_obs:
                if key not in combined_obs:
                    combined_obs[key] = full_obs[key]

        return combined_obs, combined_action


def build_controller(robot: Robot, cfg: RobotConfig) -> Controller:
    """Create the appropriate controller dynamically based on the configuration."""
    num_arms = len(cfg.arms)
    
    if num_arms == 1:
        return SingleController(robot, cfg)
    elif num_arms == 2:
        return BiController(robot, cfg)
        
    raise ValueError(f"Unsupported number of arms in configuration: {num_arms}")