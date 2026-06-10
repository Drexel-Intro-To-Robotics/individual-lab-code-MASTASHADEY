rostopic pub -1 /arm_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - joint1
    - joint2
    - joint3
    - joint4
    points:
    - positions: [0.0, -0.2, 0.2, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0]
      accelerations: [0.0, 0.0, 0.0, 0.0]
      effort: []
      time_from_start:
        secs: 3
        nsecs: 0
  path_tolerance: []
  goal_tolerance: []
  goal_time_tolerance:
    secs: 0
    nsecs: 0"
