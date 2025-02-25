source install/setup.bash

echo source ~/hexa_ws/install/setup.bash >> ~/.bashrc

colcon build

source install/setup.bash && ros2 launch hexa_description display.launch.py

source install/setup.bash && ros2 run hexa_servo servo

source install/setup.bash && ros2 run hexa_fake_slam fake_slam

lidar: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

servo: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

source install/setup.bash && ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py

ros2 topic echo /joy

ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.5, 0.0, 1.0, 1.0, 0.0, 0.0]}" --once