ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -1, max_effort: 5.0}}"

ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
    points: [{
      positions: [2.356, 0.9588, 0.05266, -3.141592653589793, 0.6646563267948964, -0.785398163397448],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }
}"

ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.1791018396615982, max_effort: 5.0}}"
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
    points: [{
      positions: [0, 0, 0, -3.141592653589793,1.5707963267948966, -3.141592653589793],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 5, nanosec: 0}
    }]
  }
}"
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
    points: [{
      positions: [-2.214297, 0.2929, -0.52, -3.141592653589793, 0.7578963267948966, 0.9272952180016123],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 10, nanosec: 0}
    }]
  }
}"
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -1.0, max_effort: 5.0}}"

ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
    points: [{
      positions: [0, 0, 0, 0, 0, 0],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }
}"