## Quick-Start Guide to Launch the Drone

### 1. Set Up the Environment
Source your workspace:

    source ~/drone_ws/install/setup.zsh

(If you're using bash, replace with setup.bash).

### 2. Launch the World and the Drone

Start Gazebo with the drone and all necessary nodes:

    ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

### 3. Take Off the Drone

Publish the takeoff command:

    ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty "{}" --once

### 4. Control the Drone

Move the drone (manual control with cmd_vel):

    ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

This moves the drone forward and increases its altitude.

Land the drone:

    ros2 topic pub /simple_drone/land std_msgs/msg/Empty "{}" --once

### 5. Monitor the Drone's Status

Check the drone's current state:

    ros2 topic echo /simple_drone/state

Possible states:

0: Landed.

1: Flying.

2: Hovering.

View sensor data and published topics (optional):

    ros2 topic echo /simple_drone/gt_pose
and
    
    ros2 topic echo /simple_drone/imu/out

### Important Notes

#### Download Gazebo Models (only once):
If you haven't already, download the common Gazebo models:

    curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip

#### Rebuild the Workspace (if you make changes):

    cd ~/drone_ws
    colcon build --packages-select-regex sjtu*
    source ~/drone_ws/install/setup.zsh

