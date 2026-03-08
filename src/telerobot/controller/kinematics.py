"""Kinematics solvers for supported robot arm types."""

import os
from lerobot.model.kinematics import RobotKinematics

from telerobot import PACKAGE_DIR


class SORobotKinematics(RobotKinematics):
    """Kinematics solver for SO-ARM101 robots.

    Loads the SO101 URDF and applies L2 regularization to prevent
    singularity-induced oscillation at full extension.

    NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo:
    https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
    """

    def __init__(self, motor_names: list[str], regularization: float = 1e-3):
        # Suppress output during placo init to hide self-collision warnings.
        # placo's C++ layer writes directly to fd 1/2, bypassing Python's sys.stdout/stderr,
        # so we must redirect at the OS file-descriptor level.
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
        # Regularization prevents singularity-induced oscillation at full extension.
        # The L2 penalty on ||dq||² keeps the QP well-conditioned so the solver
        # returns a stable, unique solution near workspace boundaries.
        self.solver.add_regularization_task(regularization)


_KINEMATICS_REGISTRY: dict[str, type[RobotKinematics]] = {
    "so100": SORobotKinematics,
}


def build_kinematics(
    arm_type: str,
    motor_names: list[str],
    regularization: float = 1e-3,
) -> RobotKinematics:
    """Build a kinematics solver for the given arm type.

    Args:
        arm_type: The robot arm type (e.g. "so100").
        motor_names: Joint/motor names for the kinematic chain.
        regularization: L2 regularization weight for the IK solver.

    Returns:
        A configured RobotKinematics instance.

    Raises:
        ValueError: If the arm type is not recognised.
    """
    kinematics_cls = _KINEMATICS_REGISTRY.get(arm_type)
    if kinematics_cls is None:
        raise ValueError(
            f"Unknown arm type '{arm_type}'. "
            f"Supported types: {', '.join(_KINEMATICS_REGISTRY)}"
        )
    return kinematics_cls(motor_names, regularization=regularization)
