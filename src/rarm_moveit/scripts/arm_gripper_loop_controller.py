#!/usr/bin/env python3
"""
Control robot arm and gripper to perform repetitive movements between positions.

This script creates a ROS 2 node that moves a robot arm between target and home positions.
"""

import os
import time
import numpy as np
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import trajectory_planing as tp


class ArmGripperLoopController(Node):
    """
    ROS 2 node to control robot arm and gripper in a looped motion sequence.
    """

    def __init__(self):
        super().__init__('arm_gripper_loop_controller')

        # Initialize motion sequences
        self.motion = []
        self.reload = False  # Set True to recalculate motion and overwrite saved data
        self.ini_motion()

        # Initialize Action Clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_action_controller/gripper_cmd'
        )

        # Joint names for arm
        self.joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        # Start the control loop
        self.control_loop_callback()

    # ----------------- Motion Initialization -----------------
    def ini_motion(self):
        """
        Initialize motion sequences and store/retrieve from file.
        """
        # Define key positions (T6 transformation matrices)
        pick_up_T6_2 = np.array([
            [1, 0, 0, 0.5],
            [0, -1, 0, -0.5],
            [0, 0, -1, 0.04],
            [0, 0, 0, 1]
        ])
        point1 = np.array([
            [1, 0, 0, 0.5],
            [0, -1, 0, -0.5],
            [0, 0, -1, 0.3],
            [0, 0, 0, 1]
        ])
        point2 = np.array([
            [1, 0, 0, -0.5],
            [0, -1, 0, -0.5],
            [0, 0, -1, 0.3],
            [0, 0, 0, 1]
        ])
        point3 = np.array([
            [1, 0, 0, -0.5],
            [0, -1, 0, 0.4],
            [0, 0, -1, 0.3],
            [0, 0, 0, 1]
        ])
        point4 = np.array([
            [1, 0, 0, 0.3],
            [0, -1, 0, 0.4],
            [0, 0, -1, 0.5],
            [0, 0, 0, 1]
        ])
        point5 = np.array([
            [1, 0, 0, 0.3],
            [0, -1, 0, 0.4],
            [0, 0, -1, 0.24],
            [0, 0, 0, 1]
        ])

        # File path to store/retrieve motion data
        package_share = get_package_share_directory("rarm_moveit")
        filename = os.path.join(package_share, "data", "data.npy")

        # Optionally delete existing file to recalculate motion
        if self.reload and os.path.exists(filename):
            os.remove(filename)
            self.get_logger().info(f"Deleted file: {filename}")

        # Load existing motion data if available
        if os.path.exists(filename):
            self.motion = np.load(filename, allow_pickle=True)
            self.get_logger().info(f"Loaded motion from {filename}")
        else:
            # Calculate motion sequences
            self.motion.append(self.calculate_motion(pick_up_T6_2, point1))
            self.motion.append(self.calculate_motion(point1, point2))
            self.motion.append(self.calculate_motion(point2, point3))
            self.motion.append(self.calculate_motion(point3, point4))
            self.motion.append(self.calculate_motion(point4, point5))
            self.motion = np.array(self.motion, dtype=object)
            np.save(filename, self.motion, allow_pickle=True)
            self.get_logger().info(f"Saved motion to {filename}")

    def calculate_motion(self, from_p_T, to_p_T):
        """
        Calculate motion trajectory between two points using trajectory planning.
        """
        point1_pos = tp.inverse_kinematics(from_p_T).tolist()
        point2_pos = tp.inverse_kinematics(to_p_T).tolist()

        if len(self.motion) > 0:
            self.get_logger().info(f"Motion count: {len(self.motion)}")
            point1_pos = tp.fix_trajectory_start(point1_pos, self.motion[-1][-1])

        return tp.main(point1_pos, point2_pos,need_plots=False).tolist()

    # ----------------- Gripper Control -----------------
    def open_gripper(self, open=True):
        """
        Open or close the gripper.
        """
        action = "Opening" if open else "Closing"
        self.get_logger().info(f'{action} the gripper')
        position = -0.05 if open else -0.01
        self.send_gripper_command(position)
        sleep_time = 5 if open else 6
        time.sleep(sleep_time)

    # ----------------- Arm Control -----------------
    def send_arm_command(self, positions: list, time_step=tp.time_step):
        """
        Send a command to move the robot arm to specified joint positions.
        """
        sec = int(time_step)
        nanosec = int((time_step - sec) * 1e9)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_command(self, position: float):
        """
        Send a command to the gripper to open or close.
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 5.0
        self.gripper_client.send_goal_async(goal_msg)

    # ----------------- Control Loop -----------------
    def control_loop_callback(self):
        """
        Execute one cycle of the control loop.
        """
        pick_up_T6_1 = np.array([
            [1, 0, 0, 0.5],
            [0, -1, 0, -0.5],
            [0, 0, -1, 0.08],
            [0, 0, 0, 1]
        ])
        point6 = np.array([
            [1, 0, 0, 0.3],
            [0, -1, 0, 0.4],
            [0, 0, -1, 0.8],
            [0, 0, 0, 1]
        ])

        # Open gripper
        self.open_gripper()

        # Move to pickup position
        self.get_logger().info('Moving to pickup position')
        self.send_arm_command(tp.inverse_kinematics(pick_up_T6_1), 4)
        time.sleep(10)
        self.send_arm_command(self.motion[0][0], 5)
        time.sleep(5)

        # Close gripper
        self.open_gripper(False)

        # Execute motion sequences
        for i, submotion in enumerate(self.motion):
            for j, theta in enumerate(submotion):
                self.get_logger().info(f'{i+1}th motion {j+1}th iteration {theta}')
                self.send_arm_command(theta)
            time.sleep(2)

        time.sleep(5)

        # Open gripper and move to final positions
        self.open_gripper()
        #time.sleep(5)
        self.get_logger().info('Moving to point6 position')
        self.send_arm_command(tp.inverse_kinematics(point6), 1)
        time.sleep(5)
        self.get_logger().info('Moving to Home position')
        self.send_arm_command([0, 0, 0, 0, 0, 0], 5)
        time.sleep(2)
        self.open_gripper(False)


def main(args=None):
    rclpy.init(args=args)
    controller = ArmGripperLoopController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down arm gripper controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
