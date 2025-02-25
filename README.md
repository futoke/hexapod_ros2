source install/setup.bash 

colcon build

source install/setup.bash && ros2 launch hexa_description display.launch.py

source install/setup.bash && ros2 run hexa_servo servo

source install/setup.bash && ros2 run hexa_fake_slam fake_slam

ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

ros2 launch ldlidar_node ldlidar_slam.launch.py

ros2 topic echo /joy

ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.5, 0.0, 1.0, 1.0, 0.0, 0.0]}" --once