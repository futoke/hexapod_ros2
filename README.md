source install/setup.bash 

colcon build

ros2 launch hexa_description display.launch.py

ros2 run hexa_servo servo

ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
ros2 launch ldlidar_node ldlidar_rviz2.launch.py
ros2 launch ldlidar_node ldlidar_slam.launch.py