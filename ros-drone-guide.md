# Drone Control and Development Guide

This document provides a brief introduction to the ROS system of topics and nodes, and explains how to start working with the SJTU Drone simulation. It also includes guidance on implementing new control algorithms, reinforcement learning, and formation control.

---

## ROS Basics: Topics and Nodes

In ROS, systems are composed of **nodes** that communicate through **topics**. This modular approach allows tasks to be separated and easily integrated.

### Components
1. **Nodes**:
   - Programs that perform specific tasks (e.g., reading sensors, controlling motors).
   - Can publish or subscribe to topics to share data.

2. **Topics**:
   - Channels used by nodes to communicate.
   - Nodes can:
     - **Publish**: Send data to a topic.
     - **Subscribe**: Receive data from a topic.

3. **Messages**:
   - Predefined data structures exchanged via topics.
   - Examples:
     - `std_msgs/msg/Empty`: Empty message.
     - `geometry_msgs/msg/Twist`: Linear and angular velocities.
     - `sensor_msgs/msg/Image`: Image data.

---

## Starting the Drone Simulation

### 1. Launch the Drone
To start the drone simulation, launch the world in Gazebo:

    ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

### 2. Test Basic Commands

##### Takeoff:

    ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty "{}" --once
##### Move the drone:

    ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

##### Land:

    ros2 topic pub /simple_drone/land std_msgs/msg/Empty "{}" --once

### 3. Monitor the Drone

Check the drone's state:

    ros2 topic echo /simple_drone/state

Possible states:

  0: Landed.
  
  1: Flying.
  
  2: Hovering.

View sensor data:

    ros2 topic echo /simple_drone/gt_pose
    ros2 topic echo /simple_drone/imu/out

---

## Implementing Custom Control Algorithms
### Step 1: Create a New Node

Write a ROS node to publish control commands. For example, a simple PID to maintain altitude:

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class PIDController(Node):
        def __init__(self):
            super().__init__('pid_controller')
            self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
            self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        msg = Twist()
        msg.linear.z = 1.0  # Example: ascend
        self.cmd_pub.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        pid = PIDController()
        rclpy.spin(pid)
        pid.destroy_node()
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()

### Step 2: Test the Node

Launch the drone in Gazebo and run the PID node. Verify that the drone responds to the commands.




