# ROS2 Sample Project

This project demonstrates ROS2 communication patterns including publisher/subscriber, services, actions, and components.

## Prerequisites

Install ROS2 using fishros:
```bash
wget http://fishros.com/install -O fishros && . fishros
```

## Project Structure

- **interface_pkg**: Custom message, service, and action definitions
- **sample_server**: Traditional ROS2 server node with publisher, service server, and action server
- **sample_client**: Traditional ROS2 client node with subscriber, service client, and action client
- **sample_server_component**: ROS2 composable component version of server (publisher functionality)
- **sample_client_component**: ROS2 composable component version of client (subscriber functionality)
- **third_party**: External dependencies (JSON library and utilities)

## Building the Project

Create and build the interface package:
```bash
ros2 pkg create --build-type ament_cmake interface_pkg --dependencies rosidl_default_generators std_msgs --license Apache-2.0
```

Build all packages:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# or for specific packages:
colcon build --packages-select interface_pkg sample_server sample_client
colcon build --packages-select sample_server_component sample_client_component
```

Source the workspace:
```bash
source install/setup.bash
```

## Running Traditional Nodes

### Basic Usage
```bash
# Run server and client
ros2 run sample_server SampleServer
ros2 run sample_client SampleClient
```

### With Parameters
```bash
# Run with custom parameters
ros2 run sample_server SampleServer --ros-args -p message_count:=3 -p server_id:=1
ros2 run sample_client SampleClient --ros-args -p message_count:=3 -p client_id:=1
```

### Using Launch Files
```bash
# Launch multiple servers
ros2 launch sample_server two_server.launch.py

# Launch multiple clients
ros2 launch sample_client two_client.launch.py
```

## Running Components

### Individual Component Launch
```bash
# Launch server component only
ros2 launch sample_server_component server_component.launch.py

# Launch client component only
ros2 launch sample_client_component client_component.launch.py
```

### Combined Component Launch
```bash
# Launch both components in the same container
ros2 launch sample_server_component components.launch.py

# Launch with different frequency patterns
ros2 launch sample_server_component cross_communication.launch.py
```

## Environment Setup

Make sure to source ROS2 and the workspace:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Monitoring

You can monitor the communication using standard ROS2 tools:
```bash
# List active topics
ros2 topic list

# Monitor message flow
ros2 topic echo /sample_topic

# Check component status
ros2 component list
```
