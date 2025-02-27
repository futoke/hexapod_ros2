source install/setup.bash

echo source ~/hexa_ws/install/setup.bash >> ~/.bashrc

colcon build

-------------------------------------------------------------------

source install/setup.bash && ros2 launch hexa_description display.launch.py

source install/setup.bash && ros2 run hexa_servo servo

source install/setup.bash && ros2 launch ldlidar_stl_ros2 ld19.launch.py

source install/setup.bash && ros2 run hexa_odom hexa_odom

source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=mapper_params_online_async.yaml

-------------------------------------------------------------------

source install/setup.bash && ros2 run hexa_fake_slam fake_slam

lidar: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

servo: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

source install/setup.bash && ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py

source install/setup.bash && ros2 launch ldlidar_stl_ros2 ld19.launch.py

ros2 topic echo /joy

ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.5, 0.0, 1.0, 1.0, 0.0, 0.0]}" --once

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once

sudo apt update

sudo apt install ros-humble-tf-transformations

sudo apt install ros-humble-slam-toolbox

ros2 pkg list | grep slam_toolbox

source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=slam_params.yaml

ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo odom base_link

ros2 topic hz /scan