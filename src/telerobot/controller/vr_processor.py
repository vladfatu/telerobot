from dataclasses import dataclass, field

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
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.phone.config_phone import PhoneOS
from lerobot.utils.rotation import Rotation


@ProcessorStepRegistry.register("map_phone_action_to_robot_action")
@dataclass
class MapVRActionToRobotAction(RobotActionProcessorStep):
    """
    Maps calibrated VR pose actions to standardized robot action inputs.

    This processor step acts as a bridge between the VR teleoperator's output
    and the robot's expected action format. It remaps the VR's 6-DoF pose
    (position and rotation) to the robot's target end-effector pose, applying
    necessary axis inversions and swaps. It also interprets platform-specific
    button presses to generate a gripper command.

    Attributes:
        platform: The operating system of the phone (iOS or Android), used
            to determine the correct button mappings for the gripper.
    """

    _enabled_prev: bool = field(default=False, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        """
        Processes the VR action dictionary to create a robot action dictionary.

        Args:
            act: The input action dictionary from the VR teleoperator.

        Returns:
            A new action dictionary formatted for the robot controller.

        Raises:
            ValueError: If 'pos' or 'rot' keys are missing from the input action.
        """
        # Pop them from the action
        enabled = bool(action.pop("enabled"))
        joystickX = action.pop("joystickX")
        pos = action.pop("pos")
        rot = action.pop("rot")

        if pos is None or rot is None:
            raise ValueError("pos and rot must be present in action")

        rot = Rotation.from_quat(rot)
        rotvec = rot.as_rotvec()  # Absolute orientation as rotvec

        # TODO remove
        # enabled = True
        # pos = [0.0, 0.0, 0.0]
        # rotvec = [0.0, 0.0, 0.0]

        gripper_vel = joystickX
        # # Map certain inputs to certain actions
        # if self.platform == PhoneOS.IOS:
        #     gripper_vel = float(inputs.get("a3", 0.0))
        # else:
        #     a = float(inputs.get("reservedButtonA", 0.0))
        #     b = float(inputs.get("reservedButtonB", 0.0))
        #     gripper_vel = (
        #         a - b
        #     )  # Positive if a is pressed, negative if b is pressed, 0 if both or neither are pressed

        # For some actions we need to invert the axis
        action["enabled"] = enabled
        action["target_x"] = -pos[2] if enabled else 0.0
        action["target_y"] = -pos[0] if enabled else 0.0
        action["target_z"] = pos[1] if enabled else 0.0
        action["target_wx"] = -rotvec[1] if enabled else 0.0
        action["target_wy"] = -rotvec[0] if enabled else 0.0
        action["target_wz"] = -rotvec[2] if enabled else 0.0
        action["gripper_vel"] = gripper_vel  # Still send gripper action when disabled
        return action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for feat in ["enabled", "pos", "rot"]:
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
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    """Build pipeline to convert VR action to ee pose action to joint action."""
    return RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[
            MapVRActionToRobotAction(),
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
