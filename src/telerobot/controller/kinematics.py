"""Kinematics solvers for supported robot arm types."""

import os
try:
    from lerobot.model.kinematics import RobotKinematics
except ImportError:
    try:
        from lerobot.kinematics import RobotKinematics
    except ImportError:
        from lerobot.common.robot_devices.kinematics import RobotKinematics

from telerobot import PACKAGE_DIR


class SORobotKinematics(RobotKinematics):
    """Kinematics solver for SO-ARM101 robots."""

    def __init__(self, motor_names: list[str], regularization: float = 1e-3):
        # Suppress output during placo init to hide self-collision warnings.
        with open(os.devnull, "w") as devnull:
            devnull_fd = devnull.fileno()
            saved_stdout_fd = os.dup(1)
            saved_stderr_fd = os.dup(2)
            try:
                os.dup2(devnull_fd, 1)
                os.dup2(devnull_fd, 2)
                super().__init__(
                    urdf_path=str(PACKAGE_DIR / "simulation" / "SO101" / "so101_new_calib.urdf"),
                    target_frame_name="gripper_frame_link",
                    joint_names=motor_names,
                )
            finally:
                os.dup2(saved_stdout_fd, 1)
                os.dup2(saved_stderr_fd, 2)
                os.close(saved_stdout_fd)
                os.close(saved_stderr_fd)
                
        # L2 penalty on ||dq||² keeps the QP well-conditioned near workspace boundaries.
        self.solver.add_regularization_task(regularization)


_KINEMATICS_REGISTRY: dict[str, type[RobotKinematics]] = {
    "so100": SORobotKinematics,
    "so101": SORobotKinematics,  # Alias for the updated arm
}


def build_kinematics(
    arm_type: str,
    motor_names: list[str],
    regularization: float = 1e-3,
) -> RobotKinematics:
    """Build a kinematics solver dynamically based on config arm_type."""
    kinematics_cls = _KINEMATICS_REGISTRY.get(arm_type.lower())
    if kinematics_cls is None:
        raise ValueError(
            f"Unknown arm type '{arm_type}'. "
            f"Supported types: {', '.join(_KINEMATICS_REGISTRY)}"
        )
    return kinematics_cls(motor_names, regularization=regularization)