wget http://fishros.com/install -O fishros && . fishros

ros2 pkg create --build-type ament_cmake interface_pkg --dependencies rosidl_default_generators std_msgs --license Apache-2.0
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release/Debug [--packages-select]



source install/setup.bash

ros2 run sample_server SampleServer
ros2 run sample_client SampleClient

ros2 run sample_server SampleServer --ros-args -p message_count:=3 -p server_id:=1
ros2 run sample_client SampleClient --ros-args -p message_count:=3 -p client_id:=1

ros2 launch sample_server two_server.launch.py
ros2 launch sample_client two_client.launch.py





source /opt/ros/jazzy/setup.bash
