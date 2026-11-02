Lab 2 - ROS2 control and Inverse Kinematics (DT512A)

  Run: ros2 launch rrbot rrbot.launch.py
       ros2 run controller_manager spawner my_state_controller --controller-manager /controller_manager
       ros2 run tf2_ros tf2_echo base_link state_controller_fk

lab02/                   Parts 3 and 4
  src/test_jacobians.cpp  Part 3
  src/test_ik.cpp          Part 4
  srv/SolveIK.srv       
  launch/test_ik.launch.py IK + robot_state_publisher (ik_joint_states) + RViz
  Run part 3: xacro $(ros2 pkg prefix rrbot)/share/rrbot/urdf/rrbot.urdf.xacro -o /tmp/rrbot.urdf
              ros2 run lab02 test_jacobians
  Run part 4: ros2 launch lab02 test_ik.launch.py
              ros2 service call /solve_ik lab02/srv/SolveIK "{frame_name: tool_link, target_pose: {position: {x: 1.148839, y: 0.2, z: 3.436532}, orientation: {x: 0.0, y: 0.389418, z: 0.0, w: 0.921061}}}"
  Parameters (test_ik): urdfPath, lambda, maxIterations, posTol, rotTol, damping
