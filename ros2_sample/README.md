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
- **ormpp_example_pkg**: Database ORM example using ORMPP library

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
colcon build --packages-select ormpp_example_pkg
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

## Running ORMPP Database Example

The ORMPP example demonstrates database operations using the ORMPP (Object-Relational Mapping for C++) library integrated with ROS2.

### Basic Usage
```bash
# Run the ORMPP example
ros2 run ormpp_example_pkg ormpp_example
```

### Database Operations

The ORMPP example performs the following database operations:
- Creates/connects to SQLite database (`example.db`)
- Demonstrates table creation and data insertion
- Shows query operations and data retrieval
- Illustrates ORM mapping between C++ objects and database tables

### Database File Verification

After running the ORMPP example, you can verify the database operations using the following methods:

#### Method 1: Using SQLite3 Command Line Tool
```bash
# Install sqlite3 if not already installed
sudo apt-get install sqlite3

# Open the database file
sqlite3 example.db

# List all tables
.tables

# Show table schema
.schema

# Query data from tables
SELECT * FROM your_table_name;

# Exit sqlite3
.exit
```

### Common Database Operations Verification

```bash
# Check database integrity
sqlite3 example.db "PRAGMA integrity_check;"

# View database stats
sqlite3 example.db "PRAGMA database_list;"

# Export database to SQL file
sqlite3 example.db ".dump" > database_backup.sql

# Import database from SQL file
sqlite3 new_database.db < database_backup.sql
```

## Running Components

ROS2 components allow multiple nodes to run in the same process, improving performance through intra-process communication and reduced memory overhead.

### Manual Component Loading

#### Method 1: Using Component Container
Start a component container first, then load components into it:

```bash
# Start a component container
ros2 run rclcpp_components component_container

# In another terminal, load server component
ros2 component load /ComponentManager sample_server_component sample_server_component::SampleServerComponent

# Load client component
ros2 component load /ComponentManager sample_client_component sample_client_component::SampleClientComponent

# List loaded components
ros2 component list

# Unload components when done
ros2 component unload /ComponentManager 1 2
```

#### Method 2: Using Standalone Component Manager
```bash
# Run component manager with immediate component loading
ros2 run rclcpp_components component_container --ros-args -r __node:=my_container

# Load components with parameters
ros2 component load /my_container sample_server_component sample_server_component::SampleServerComponent -p publish_frequency:=5.0 -p topic_name:=custom_topic
```

### Launch File Based Component Loading

### Individual Component Launch
```bash
# Launch server component only
ros2 launch sample_server_component server_component.launch.py

# Launch client component only
ros2 launch sample_client_component client_component.launch.py
```

### Combined Component Launch
```bash
# Launch both components in the same container (enables intra-process communication)
ros2 launch sample_server_component components.launch.py

# Launch with different frequency patterns
ros2 launch sample_server_component cross_communication.launch.py
```

### Component vs Traditional Node Comparison

| Feature | Traditional Nodes | Components |
|---------|------------------|------------|
| Process isolation | Each node in separate process | Multiple nodes in single process |
| Memory usage | Higher (separate processes) | Lower (shared process) |
| Communication | Inter-process (slower) | Intra-process (faster) |
| Debugging | Easier to debug individual nodes | Shared process debugging |
| Fault isolation | Node crash doesn't affect others | Component crash affects container |

### Component Loading with Parameters

You can customize component behavior by passing parameters:

```bash
# Load server component with custom parameters
ros2 component load /ComponentManager sample_server_component sample_server_component::SampleServerComponent -p publish_frequency:=10.0 -p topic_name:=high_freq_topic

# Load client component with custom ID
ros2 component load /ComponentManager sample_client_component sample_client_component::SampleClientComponent -p client_id:=99 -p topic_name:=high_freq_topic
```

### Monitoring Component Status

```bash
# List all component containers
ros2 component list

# Show detailed component information
ros2 component types

# Monitor component container logs
ros2 run rclcpp_components component_container --ros-args --log-level debug
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
