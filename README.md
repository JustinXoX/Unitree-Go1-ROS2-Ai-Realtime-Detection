# Unitree-Go1-ROS2-AI-Realtime-Detection

## Project Overview

This project provides a comprehensive simulation setup for the Unitree GO1 robot using ROS 2 and Gazebo. It covers launching simulations, activating robot control interfaces, running navigation stacks with visualization in RViz, real-time violation detection using AI, and a web-based control panel for managing the robot.

## Prerequisites

Before you begin, ensure you have the following software installed:

- **ROS 2 Humble**: The latest LTS version of ROS 2, providing robust middleware support for robotics applications.
- **Gazebo** (Version 11 recommended): A 3D robotics simulator used to visualize and simulate the robot environment.
- **Unitree robot software and dependencies**: Specific software required to interact with and control the GO1 robot.
- **Python 3.x and necessary libraries**: Used for AI detection scripts and other utilities.
- **Apache Web Server**: Used to host the HTML-based control panel.
- **rosbridge_server**: A ROS package that provides a JSON API to ROS functionality for communication over WebSocket.

## Installation Guide

Follow these steps to set up the project environment:

### Step 1: Install ROS 2 Humble

1. **Update your system**:
   Ensure your system is up-to-date to avoid compatibility issues.
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. **Add the ROS 2 repository and install ROS 2 Humble**:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. **Source the ROS 2 environment**:
   Add ROS 2 environment setup to your shell configuration to enable ROS commands in your terminal.
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Step 2: Install Gazebo

1. **Install Gazebo 11**:
   Gazebo is required for simulating the robot in a 3D environment.
   ```bash
   sudo apt install gazebo11
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. **Verify the installation**:
   Open a terminal and type `gazebo` to ensure Gazebo launches correctly.

### Step 3: Set Up the Unitree Robot Software and Dependencies

1. **Clone the GO1 robot project repository**:
   ```bash
   git clone https://github.com/<your-github-username>/go1_robot_project.git
   cd go1_robot_project
   ```

2. **Install ROS dependencies**:
   The `rosdep` tool installs the required ROS packages specified by the project, ensuring that all dependencies are correctly set up.
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the ROS 2 workspace**:
   Use `colcon`, the recommended build tool for ROS 2, to compile the project packages. `--symlink-install` creates symlinks instead of copying files, which helps during development.
   ```bash
   colcon build --symlink-install
   ```

### Step 4: Install Python and Necessary Libraries

Python is essential for running AI scripts and additional utilities.

1. **Ensure Python 3.x is installed**:
   ```bash
   sudo apt install python3 python3-pip
   ```

2. **Install required Python libraries**:
   - **opencv-python**: Used for computer vision tasks, such as image processing and real-time object detection.
   - **requests**: A simple HTTP library for making requests to web services, often used in machine learning projects for fetching models or data.
   
   Install these libraries using pip:
   ```bash
   pip3 install opencv-python requests
   ```

### Step 5: Configure AI Detection Environment

1. **Set the `ROBOFLOW_API_KEY` environment variable**:
   This environment variable is used by the AI detection script to authenticate API requests to Roboflow, a machine learning platform for computer vision.
   ```bash
   export ROBOFLOW_API_KEY="YourAPIKey"
   ```

### Step 6: Install and Configure Apache Web Server

1. **Install Apache**:
   Apache is used to host the HTML file for the robot control panel, making it accessible through a web browser.
   ```bash
   sudo apt install apache2
   ```

2. **Start and enable Apache**:
   Ensure Apache runs automatically on boot and is currently active.
   ```bash
   sudo systemctl start apache2
   sudo systemctl enable apache2
   ```

3. **Edit the HTML file**:
   The control panel HTML file should be placed in the Apache root directory (`/var/www/html/`). Modify the file using a text editor:
   ```bash
   sudo gedit /var/www/html/index.html
   ```

4. **Modify the IP address in the HTML**:
   Update the line in your HTML file that specifies the connection to the ROS server. Change the IP to match your ROS 2 WebSocket server’s IP address.
   ```html
   var ros = new ROSLIB.Ros({
       url: 'ws://192.168.3.203:9090'  // Update this IP address to your ROS 2 server's IP address
   });
   ```

### Step 7: Launch ROS 2 Components

1. **Launch the Gazebo Simulation**:
   Start the simulation environment in a new terminal window:
   ```bash
   ros2 launch go1_gazebo spawn_go1.launch.py
   ```

2. **Activate the Robot Control Interface**:
   In another terminal window, start the robot control interface to manage the GO1 robot's states:
   ```bash
   ros2 run unitree_guide2 junior_ctrl
   ```

3. **Start the Navigation Stack**:
   Initialize the navigation stack to allow autonomous navigation:
   ```bash
   ros2 launch go1_navigation navigation_new.launch.py
   ```

4. **Launch the ROS 2 WebSocket Server**:
   Start `rosbridge_server` to enable WebSocket communication between ROS 2 and the web-based control panel:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

### Step 8: Using the Web-Based Control Panel

1. **Access the Control Panel**:
   Open a web browser and go to `http://localhost` or the IP address of your Apache server if accessed remotely.

2. **Control the Robot**:
   Use the control panel buttons to send commands to the robot, such as moving forward, backward, left, or right.

3. **Adjust Speed and Set Navigation Goals**:
   Adjust the robot's speed using the slider and set navigation goals by clicking the "Set Navigation Goal" button.

4. **Stop the Robot**:
   Press the "Stop" button to immediately halt the robot’s movements.

### Step 9: Run AI Detection Script

1. **Verify the environment variable**:
   Make sure `ROBOFLOW_API_KEY` is correctly set:
   ```bash
   echo $ROBOFLOW_API_KEY
   ```

2. **Run the AI detection script**:
   Execute the script that uses the AI camera to detect violations in real-time:
   ```bash
   python3 /path/to/AiDetection.py
   ```

### Troubleshooting and Tips

- **Web page does not load**: Confirm that Apache is running (`sudo systemctl status apache2`), and check that the HTML file is correctly placed in `/var/www/html/`.
- **ROS connection fails**: Verify that the WebSocket server is active and that the IP address in your HTML file is correct.
---
