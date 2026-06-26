# Warehouse-Patrol-ROS2-Simulation
A simulation where a robot navigates a warehouse to reach certain destinations

Following steps are to be done using Ubuntu terminal iwth ROS2 installed with a ros2_ws and src folder:

  Prepare the simulation with the command:
  cd ~/ros2_ws
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --packages-select warehouse_patrol
  source install/setup.bash

  Run the RViz application with the command:
  ros2 launch warehouse_patrol warehouse_patrol.launch.py

  Run the full warehouse simulation on gazebo with the command:
  ros2 launch warehouse_patrol warehouse_patrol.launch.py \
  gz_args:="-r $(ros2 pkg prefix warehouse_patrol)/share/warehouse_patrol/worlds/warehouse.sdf" \
  launch_rviz:=true
