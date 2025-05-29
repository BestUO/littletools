wget http://fishros.com/install -O fishros && . fishros

ros2 pkg create --build-type ament_cmake interface_pkg --dependencies rosidl_default_generators std_msgs --license Apache-2.0
colcon build [--packages-select]



source install/setup.bash
ros2 run sample_server SampleServer
ros2 run sample_client SampleClient


source /opt/ros/jazzy/setup.bash
