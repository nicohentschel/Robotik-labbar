Lab 1 - Coordinate Frames (DT512A)

Put the lab01 folder in workspace/src of the robotteknik repo and build:
  colcon build --packages-select lab01

lab01/
  src/shortest_transform.cpp   Part 2: Shortest Transform (15 p)
  src/test_fk.cpp              Part 3: Forward kinematics with KDL (15 p)
  launch/test_fk.launch.py     rrbot (RViz + RQT) + xacro -> /tmp/rrbot.urdf + test_fk

Run part 2: ros2 run lab01 shortest_transform
            rviz2   (Fixed Frame: world, Add -> TF)
  Parameters: numSteps, worldFrame

Run part 3: ros2 launch lab01 test_fk.launch.py
            ros2 run tf2_ros tf2_echo base_link my_fk
            ros2 run tf2_ros tf2_echo base_link tool_link   (should give the same values)
  Parameters: urdfPath, baseFrame, tipFrame, fkFrame
