import argparse
import time
import cv2
import numpy as np
import copy
import threading

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
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
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from telerobot import PACKAGE_DIR
from telerobot.config import load_robot
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
    kin.solver.add_regularization_task(2e-3)
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


def main():
    parser = argparse.ArgumentParser(description="Telerobot — VR teleoperation for SO-ARM101")
    parser.add_argument(
        "-c", "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_PATH})",
    )
    args = parser.parse_args()

    duo_robot, cfg = load_robot(args.config)

    teleop_device = VRHeadset()

    # Initialize WebRTC camera server with HTTPS
    use_https = True  # Set to False for HTTP
    cert_file = "ssl_cert/server.crt"
    key_file = "ssl_cert/server.key"

    camera_server = create_camera_server(
        duo_robot.cameras.keys(),
        use_https=use_https,
        cert_file=cert_file,
        key_file=key_file
    )

    # Start camera server in background thread
    server_thread = threading.Thread(target=camera_server.run_in_thread, daemon=True)
    server_thread.start()

    if use_https:
        print("🔒 HTTPS WebRTC camera server started on https://0.0.0.0:8765")
        print("📱 Access from Quest 3: https://YOUR_IP:8765")
    else:
        print("🎥 HTTP WebRTC camera server started on http://0.0.0.0:8765")

    processors = {
        "left_arm": get_vr_to_arm_processor(list(duo_robot.left_arm.bus.motors.keys())),
        "right_arm": get_vr_to_arm_processor(list(duo_robot.right_arm.bus.motors.keys())),
        "has_initial_position": True
    }

    # Connect to the robot and teleoperator
    duo_robot.connect()
    teleop_device.connect()

    # Init rerun viewer
    init_rerun(session_name="vr_lerobot_duo_teleop")

    if not duo_robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    initial_right_arm_obs = duo_robot.right_arm.get_observation()
    initial_left_arm_obs = duo_robot.left_arm.get_observation()

    def reset_robot_to_initial_position():
        print("Resetting robot to initial position...")
        _ = duo_robot.right_arm.send_action(initial_right_arm_obs)
        _ = duo_robot.left_arm.send_action(initial_left_arm_obs)

        processors["left_arm"] = get_vr_to_arm_processor(list(duo_robot.left_arm.bus.motors.keys()))
        processors["right_arm"] = get_vr_to_arm_processor(list(duo_robot.right_arm.bus.motors.keys()))
        processors["has_initial_position"] = True

    print("Starting teleop loop. Move your phone to teleoperate the robot...")
    while True:
        t0 = time.perf_counter()

        # Get robot observation
        right_arm_obs = duo_robot.right_arm.get_observation()
        left_arm_obs = duo_robot.left_arm.get_observation()
        # robot_obs = {'shoulder_pan.pos': 1.3186813186813187, 'shoulder_lift.pos': -20.703296703296704, 'elbow_flex.pos': 8.131868131868131, 'wrist_flex.pos': 60.35164835164835, 'wrist_roll.pos': 8.483516483516484, 'gripper.pos': 1.2303485987696514}

        # Capture and stream camera frames
        try:
            # Get frames from cameras
            left_wrist_frame = duo_robot.cameras["left_wrist"].async_read(timeout_ms=50)
            right_wrist_frame = duo_robot.cameras["right_wrist"].async_read(timeout_ms=50)
            main_frame = duo_robot.cameras["main"].async_read(timeout_ms=500)

            # Update WebRTC streams
            if left_wrist_frame is not None:
                camera_server.update_camera_frame("left_wrist", left_wrist_frame)
            if right_wrist_frame is not None:
                camera_server.update_camera_frame("right_wrist", right_wrist_frame)
            if main_frame is not None:
                camera_server.update_camera_frame("main", main_frame)

        except Exception as e:
            print(f"Error capturing camera frames: {e}")

        # Get teleop action
        vr_obs = teleop_device.last_observation

        if vr_obs is None:
            # print("No VR observation received yet.")
            # log_rerun_data(observation=left_arm_obs, action=None)
            log_rerun_data(observation=duo_robot.get_observation(), action=None)
        elif vr_obs['reset'] and not processors["has_initial_position"]:
            reset_robot_to_initial_position()
        else:
            print("VR Observation: ", vr_obs)

            right_controller_obs = copy.deepcopy(vr_obs["right"])
            # print(f"VR Observation: {right_controller_obs}")

            if right_controller_obs["enabled"]:
                processors["has_initial_position"] = False
                print(f"Right Arm VR Position: {right_controller_obs['pos']}")
            else:
                print("Right controller not enabled.")

            right_joint_action = processors["right_arm"]((right_controller_obs, right_arm_obs))
            _ = duo_robot.right_arm.send_action(right_joint_action)


            left_controller_obs = copy.deepcopy(vr_obs["left"])
            # print(f"VR Observation: {left_controller_obs}")

            if left_controller_obs["enabled"]:
                processors["has_initial_position"] = False
                print(f"Left Arm VR Position: {left_controller_obs['pos']}")
            else:
                print("Left controller not enabled.")

            left_joint_action = processors["left_arm"]((left_controller_obs, left_arm_obs))
            _ = duo_robot.left_arm.send_action(left_joint_action)

            # # Add prefixes back
            # prefixed_send_action_left = {f"left_{key}": value for key, value in left_joint_action.items()}
            # prefixed_send_action_right = {f"right_{key}": value for key, value in right_joint_action.items()}

            # joint_action = {**prefixed_send_action_left, **prefixed_send_action_right}

            # _ = duo_robot.send_action(joint_action)

        # busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

        ## TODO remove - for testing only
        precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()
