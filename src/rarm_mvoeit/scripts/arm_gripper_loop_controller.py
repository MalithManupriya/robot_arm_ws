#!/usr/bin/env python3
"""
Control robot arm and gripper to perform repetitive movements between positions.

This script creates a ROS 2 node that moves a robot arm between target and home positions,
coordinating with gripper actions (open/close) at each position. The movement happens
in a continuous loop.

Action Clients:
    /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory):
        Commands for controlling arm joint positions
    /gripper_action_controller/gripper_cmd (control_msgs/GripperCommand):
        Commands for opening and closing the gripper

:author: Addison Sears-Collins
:date: November 15, 2024
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import trajectory_planing as tp
class ArmGripperLoopController(Node):
    """
    A ROS 2 node for controlling robot arm movements and gripper actions.

    This class creates a simple control loop that:
    1. Moves the arm to a target position
    2. Closes the gripper
    3. Returns the arm to home position
    4. Opens the gripper
    """
    def calculate_motion(self,from_p_T,to_p_T):
        point1_pos=tp.inverse_kinematics(from_p_T).tolist()
        point2_pos=tp.inverse_kinematics(to_p_T).tolist()
        if len(self.motion)>0:
            print(f"motio count is {len(self.motion)}")
            point1_pos=tp.fix_trajectory_start(point1_pos,self.motion[-1][-1])
            #point2_pos=tp.fix_trajectory_start(point2_pos,point1_pos)
        return tp.main(point1_pos,point2_pos)
    
    def open_gripper(self,open=True):
        if open:
            self.get_logger().info('Opening the gripper')
            self.send_gripper_command(-1.2)  # Open the gripper
            time.sleep(5)
        else :
            self.get_logger().info('Closing the gripper')
            self.send_gripper_command(-0.1)  # Close the gripper
            time.sleep(10)
    
    def control_loop_callback(self) -> None:
        """
        Execute one cycle of the control loop.

        This method performs the following sequence:
        1. Open the gripper
        2. Move to pickup location
        3. Close the Gripper
        4. Go to pick down location
        5. open the gripper
        """
        pick_up_T6=np.array([
            [1,     0,      0,       0.5    ],
            [0,    -1,      0,      -0.5    ],
            [0,     0,     -1,      0.05    ],
            [0,     0,      0,         1    ]
        ])
        point1=np.array([
            [1,     0,      0,       0.5    ],
            [0,    -1,      0,      -0.5    ],
            [0,     0,     -1,      0.3    ],
            [0,     0,      0,         1    ]
        ])
        point2=np.array([
            [1,     0,      0,      -0.5    ],
            [0,    -1,      0,      -0.5    ],
            [0,     0,     -1,      0.3    ],
            [0,     0,      0,         1    ]
        ])
        point3=np.array([
            [1,     0,      0,      -0.5    ],
            [0,    -1,      0,      0.4    ],
            [0,     0,     -1,      0.3    ],
            [0,     0,      0,         1    ]
        ])
        point4=np.array([
            [1,     0,      0,      0.3    ],
            [0,    -1,      0,      0.4    ],
            [0,     0,     -1,      0.3    ],
            [0,     0,      0,         1    ]
        ])
        point5=np.array([
            [1,     0,      0,      0.3    ],
            [0,    -1,      0,      0.4    ],
            [0,     0,     -1,      0.25    ],
            [0,     0,      0,         1    ]
        ])

        self.motion=[]
        self.motion.append(self.calculate_motion(pick_up_T6,point1))
        self.motion.append(self.calculate_motion(point1,point2))
        self.motion.append(self.calculate_motion(point2,point3))
        self.motion.append(self.calculate_motion(point3,point4))
        self.motion.append(self.calculate_motion(point4,point5))

        self.open_gripper()

        self.get_logger().info('Moving to pickup position')
        self.send_arm_command(self.motion[0][0] ,5)
        time.sleep(10)

        self.open_gripper(False)

        for i,submotion in enumerate(self.motion):
            for j,theta in enumerate(submotion):
                self.get_logger().info(f'{i+1}th motion {j+1}th iteration {theta}')
                self.send_arm_command(theta)
            time.sleep(2)

        time.sleep(1)

        self.open_gripper()
        
        self.get_logger().info('Moving to Home position')
        self.send_arm_command([0,0,0,0,0,0] ,5)
        time.sleep(10)

    def __init__(self):
        """
        Initialize the node and set up action clients for arm and gripper control.

        Sets up two action clients:
        - One for controlling arm movements
        - One for controlling gripper open/close actions
        Also defines the positions for movement and starts a timer for the control loop.
        """
        super().__init__('arm_gripper_loop_controller')

        # Set up arm trajectory action client for arm movement control
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Set up gripper action client for gripper control
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd'
        )

        # Wait for both action servers to be available
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        # List of joint names for the robot arm
        self.joint_names = [
            'J1', 'J2', 'J3',
            'J4', 'J5', 'J6'
        ]

        #self.pickdown_pos=[0, 0, 0, -3.141592653589793,1.5707963267948966, -3.141592653589793]
        # Create timer that triggers the control loop quickly after start (0.1 seconds)
        #self.create_timer(0.1, self.control_loop_callback)
        self.control_loop_callback()

    def send_arm_command(self, positions: list,time_step=tp.time_step) -> None:
        """
        Send a command to move the robot arm to specified joint positions.

        Args:
            positions (list): List of 6 joint angles in radians
        """
        sec = int(time_step)
        nanosec = int((time_step - sec) * 1e9)
        # Create a trajectory point with the target positions
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=sec,nanosec=nanosec)  # Allow 2 seconds for movement

        # Create and send the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_command(self, position: float) -> None:
        """
        Send a command to the gripper to open or close.

        Args:
            position (float): Position value for gripper (0.0 for open, -0.7 for closed)
        """
        # Create and send the gripper command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 5.0

        self.gripper_client.send_goal_async(goal_msg)

    


def main(args=None):
    """
    Initialize and run the arm gripper control node.

    Args:
        args: Command-line arguments (default: None)
    """
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
