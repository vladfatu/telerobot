from dataclasses import dataclass, field
import time

from lerobot.configs.types import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    ProcessorStepRegistry,
    RobotAction,
    RobotActionProcessorStep,
    RobotObservation,
    RobotProcessorPipeline,
)
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)

try:
    from lerobot.robots.so_follower.robot_kinematic_processor import (
        EEBoundsAndSafety,
        EEReferenceAndDelta,
        GripperVelocityToJoint,
        InverseKinematicsEEToJoints,
    )
except ImportError:
    # v0.5+ sometimes renames this directory
    from lerobot.robots.so100_follower.robot_kinematic_processor import (
        EEBoundsAndSafety,
        EEReferenceAndDelta,
        GripperVelocityToJoint,
        InverseKinematicsEEToJoints,
    )
from lerobot.utils.rotation import Rotation


@ProcessorStepRegistry.register("map_vr_action_to_robot_action")
@dataclass
class MapVRActionToRobotAction(RobotActionProcessorStep):
    """
    Maps VR pose actions to robot end-effector deltas.

    Control scheme:
    - Grip/enabled: sculpting / relative pose control.
    - Trigger: gripper open/close velocity.
    - Thumbstick X: wrist/gripper roll while sculpting.
    - A/B buttons: base rotation is handled later in controller.py.

    Position and rotation remain trigger-latched relative to grip engagement,
    preserving the mannequin/sculpting behavior.
    """

    _enabled_prev: bool = field(default=False, init=False, repr=False)
    _pos_reference: tuple[float, float, float] | None = field(default=None, init=False, repr=False)
    _rotvec_reference: tuple[float, float, float] | None = field(default=None, init=False, repr=False)

    position_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_trigger_mode: str = "Press Trigger to Close"

    # Thumbstick-controlled wrist roll.
    # This accumulates while grip/sculpting is enabled, making the thumbstick
    # behave like a slow roll velocity control.
    wrist_roll_rate: float = 0.7  # radians/second at full thumbstick
    _wrist_roll_offset: float = field(default=0.0, init=False, repr=False)
    _last_roll_time: float | None = field(default=None, init=False, repr=False)

    def _deadzone(self, value: float, threshold: float = 0.15) -> float:
        return 0.0 if abs(value) < threshold else value

    def update_runtime_settings(
        self,
        position_scale: float | None = None,
        rotation_scale: float | None = None,
        gripper_trigger_mode: str | None = None,
    ) -> bool:
        """Update live teleop settings without rebuilding the processor pipeline.

        If scale changes while sculpting is active, clear the pose latch so the
        next VR action re-latches at the current hand pose instead of causing a jump.
        """
        changed = False
        scale_changed = False

        if position_scale is not None:
            new_value = float(position_scale)
            if self.position_scale != new_value:
                self.position_scale = new_value
                changed = True
                scale_changed = True

        if rotation_scale is not None:
            new_value = float(rotation_scale)
            if self.rotation_scale != new_value:
                self.rotation_scale = new_value
                changed = True
                scale_changed = True

        if gripper_trigger_mode is not None:
            if gripper_trigger_mode not in ("Press Trigger to Close", "Press Trigger to Open"):
                gripper_trigger_mode = "Press Trigger to Close"

            if self.gripper_trigger_mode != gripper_trigger_mode:
                self.gripper_trigger_mode = gripper_trigger_mode
                changed = True

        if scale_changed:
            self._pos_reference = None
            self._rotvec_reference = None
            self._wrist_roll_offset = 0.0
            self._last_roll_time = None
            self._enabled_prev = False

        return changed

    def action(self, action: RobotAction) -> RobotAction:
        """
        Convert VR controller pose into relative robot end-effector deltas.

        Grip/enabled controls sculpting. Trigger controls gripper velocity.
        Thumbstick X controls wrist roll while sculpting.
        """
        enabled = bool(action.pop("enabled"))
        joystickX = float(action.pop("joystickX", 0.0) or 0.0)

        # New frontend fields.
        trigger = float(action.pop("trigger", 0.0) or 0.0)
        action.pop("buttonA", None)
        action.pop("buttonB", None)
        action.pop("buttonsRaw", None)

        pos = action.pop("pos")
        rot = action.pop("rot")

        if pos is None or rot is None:
            raise ValueError("pos and rot must be present in action")

        mapped_pos = (
            -float(pos[2]),
            -float(pos[0]),
            float(pos[1]),
        )

        rot = Rotation.from_quat(rot)
        rotvec = rot.as_rotvec()

        mapped_rotvec = (
            -float(rotvec[1]),
            -float(rotvec[0]),
            -float(rotvec[2]),
        )

        now = time.time()
        roll_input = self._deadzone(joystickX)

        # Grip still controls sculpting.
        # Thumbstick roll is allowed to control the wrist even when grip is not held.
        control_enabled = enabled or abs(roll_input) > 0.0

        if control_enabled:
            # Rising edge: grip/sculpting was just pressed.
            if (
                not self._enabled_prev
                or self._pos_reference is None
                or self._rotvec_reference is None
            ):
                self._pos_reference = mapped_pos
                self._rotvec_reference = mapped_rotvec
                self._wrist_roll_offset = 0.0
                self._last_roll_time = now

            target_x = (mapped_pos[0] - self._pos_reference[0]) * self.position_scale
            target_y = (mapped_pos[1] - self._pos_reference[1]) * self.position_scale
            target_z = (mapped_pos[2] - self._pos_reference[2]) * self.position_scale

            target_wx = (mapped_rotvec[0] - self._rotvec_reference[0]) * self.rotation_scale
            target_wy = (mapped_rotvec[1] - self._rotvec_reference[1]) * self.rotation_scale
            target_wz = (mapped_rotvec[2] - self._rotvec_reference[2]) * self.rotation_scale

            # Thumbstick X now controls wrist/gripper roll.
            # Use time-based accumulation so the speed is stable even if loop FPS changes.
            if self._last_roll_time is None:
                self._last_roll_time = now

            dt = max(0.0, min(now - self._last_roll_time, 0.1))
            self._last_roll_time = now

            self._wrist_roll_offset += roll_input * self.wrist_roll_rate * dt
            target_wz += self._wrist_roll_offset

        else:
            # When not sculpting, clear pose latch and wrist roll accumulation.
            self._pos_reference = None
            self._rotvec_reference = None
            self._wrist_roll_offset = 0.0
            self._last_roll_time = None

            target_x = 0.0
            target_y = 0.0
            target_z = 0.0
            target_wx = 0.0
            target_wy = 0.0
            target_wz = 0.0

        self._enabled_prev = control_enabled

        # Trigger controls gripper open/close.
        #
        # Because different gripper sign conventions exist, expose this as a GUI preference:
        # - "Press Trigger to Close": trigger pressed should close the gripper.
        # - "Press Trigger to Open":  trigger pressed should open the gripper.
        #
        # This remains bidirectional analog:
        # released = one direction, half press = neutral-ish, pressed = opposite direction.
        if self.gripper_trigger_mode == "Press Trigger to Open":
            gripper_vel = (trigger * 2.0) - 1.0
        else:
            gripper_vel = 1.0 - (trigger * 2.0)

        action["enabled"] = control_enabled
        action["target_x"] = target_x
        action["target_y"] = target_y
        action["target_z"] = target_z
        action["target_wx"] = target_wx
        action["target_wy"] = target_wy
        action["target_wz"] = target_wz
        action["gripper_vel"] = gripper_vel
        return action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for feat in [
            "enabled",
            "pos",
            "rot",
            "trigger",
            "buttonA",
            "buttonB",
            "buttonsRaw",
        ]:
            features[PipelineFeatureType.ACTION].pop(feat, None)

        for feat in [
            "enabled",
            "target_x",
            "target_y",
            "target_z",
            "target_wx",
            "target_wy",
            "target_wz",
            "gripper_vel",
        ]:
            features[PipelineFeatureType.ACTION][f"{feat}"] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )

        return features


def build_vr_to_arm_processor(
    motor_names: list[str],
    kinematics_solver: RobotKinematics,
    end_effector_step_sizes: dict[str, float],
    end_effector_bounds: dict[str, list[float]],
    max_ee_step_m: float,
    gripper_speed_factor: float,
    position_scale: float = 1.0,
    rotation_scale: float = 1.0,
    gripper_trigger_mode: str = "Press Trigger to Close",
    **_ignored,
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    """Build pipeline to convert VR action to ee pose action to joint action."""
    return RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[
            MapVRActionToRobotAction(
                position_scale=position_scale,
                rotation_scale=rotation_scale,
                gripper_trigger_mode=gripper_trigger_mode,
            ),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes=end_effector_step_sizes,
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds=end_effector_bounds,
                max_ee_step_m=max_ee_step_m,
            ),
            GripperVelocityToJoint(
                speed_factor=gripper_speed_factor,
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