# drone_bringup

## Package Description

The `drone_bringup` package is responsible for launching and initializing the quadcopter simulation and ROS 2 nodes. It provides launch files to start the Gazebo simulator with the drone model and all necessary ROS 2 nodes for drone control and monitoring.

## Running the Launch File

To launch the drone simulation with all required nodes, use:

```bash
ros2 launch drone_bringup spawn_gz.launch.py
```


## Publishing to Topics

To publish commands or data to drone topics, use the ROS 2 `pub` command:

### Example: Publish to Thrust Command Topic

```bash
ros2 topic pub /x500/command/motor_speed_cmd actuator_msgs/msg/Actuators   "{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' }, position: [], velocity: [600.0, 600.0, 600.0, 600.0], normalized: [] }"
```

