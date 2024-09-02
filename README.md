# Unitree-Go1-ROS2-Ai-Realtime-Detection
 
## Project Overview

This project provides a comprehensive simulation setup for the Unitree GO1 robot using ROS 2 and Gazebo. It covers launching simulations, activating robot control interfaces, running navigation stacks with visualization in RViz, and real-time violation detection using AI.

## Prerequisites

Before you begin, ensure you have the following software installed:

- **ROS 2** (Foxy or later version): A popular middleware framework for robot software development.
- **Gazebo** (Version 11 recommended): A 3D robotics simulator used to visualize and simulate the robot environment.
- **Unitree robot software and dependencies**: Required to interact with and control the GO1 robot.
- **Python 3.x and necessary libraries**: Used for AI detection scripts and other utilities.

## Installation Guide

Follow these steps to set up the project environment:

### Step 1: Install ROS 2

1. **Update your system**: Before installing ROS 2, ensure your system is up-to-date.
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. **Add the ROS 2 repository to your sources list**:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

3. **Install ROS 2 Foxy**:
   ```bash
   sudo apt install ros-foxy-desktop
   ```

4. **Source the ROS 2 environment**: To use ROS 2 commands and packages, source the setup file in your `.bashrc`:
   ```bash
   echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Step 2: Install Gazebo

1. **Install Gazebo 11**:
   ```bash
   sudo apt install gazebo11
   sudo apt install ros-foxy-gazebo-ros-pkgs
   ```

2. **Verify the installation**:
   Open a terminal and type `gazebo` to ensure Gazebo opens correctly. Close the simulator once verified.

### Step 3: Set Up the Unitree Robot Software and Dependencies

1. **Clone the GO1 robot project repository**:
   ```bash
   git clone https://github.com/<your-github-username>/go1_robot_project.git
   cd go1_robot_project
   ```

2. **Install ROS dependencies**:
   Use `rosdep` to install all the required dependencies for the ROS 2 packages in your project.
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the ROS 2 workspace**:
   Use `colcon`, the recommended build tool for ROS 2, to build your workspace.
   ```bash
   colcon build --symlink-install
   ```

### Step 4: Install Python and Necessary Libraries

1. **Ensure Python 3.x is installed**:
   ```bash
   sudo apt install python3 python3-pip
   ```

2. **Install required Python libraries for AI detection**:
   Use `pip` to install necessary Python libraries, such as `opencv-python` and `requests`:
   ```bash
   pip3 install opencv-python requests
   ```

### Step 5: Configure AI Detection Environment

1. **Set the `ROBOFLOW_API_KEY` environment variable**:
   The AI detection script requires an API key from Roboflow. Set this environment variable with your API key.

   For Linux and macOS:
   ```bash
   export ROBOFLOW_API_KEY="YourAPIKey"
   ```

   For Windows PowerShell:
   ```bash
   $env:ROBOFLOW_API_KEY="YourAPIKey"
   ```

   Replace `YourAPIKey` with your actual Roboflow API key.

### Step 6: Running the Simulation and Control

1. **Launch the Gazebo Simulation**:
   Open a new terminal and run:
   ```bash
   ros2 launch go1_gazebo spawn_go1.launch.py
   ```
   This command initializes the Gazebo simulation environment with the GO1 robot.

2. **Activate the Robot Control Interface**:
   In a second terminal window, activate the robot control interface by running:
   ```bash
   ros2 run unitree_guide2 junior_ctrl
   ```
   Make sure the controllers are fully loaded in Gazebo before running this command.

3. **Start the Navigation Stack**:
   In a third terminal window, launch the navigation stack to enable autonomous navigation:
   ```bash
   ros2 launch go1_navigation navigation_new.launch.py
   ```

### Step 7: Run AI Detection Script

1. **Ensure the environment variable is set correctly**:
   Before running the AI detection script, confirm the `ROBOFLOW_API_KEY` is set by typing `echo $ROBOFLOW_API_KEY` (Linux/macOS) or `$env:ROBOFLOW_API_KEY` (Windows PowerShell).

2. **Run the AI detection script**:
   ```bash
   python3 /path/to/AiDetection.py
   ```
   Replace `/path/to/` with the actual path where your `AiDetection.py` script is located.

### Final Check

1. **Verify that all components are working**:
   - The Gazebo simulator should display the GO1 robot.
   - The control interface should respond to commands.
   - The navigation stack should be active in RViz.
   - The AI detection script should detect actions in real-time.

2. **Troubleshooting**:
   - If any component fails, check for error messages in the terminal and ensure all dependencies and software are correctly installed.

## Contributions

We welcome contributions! Please fork this repository, make your changes, and submit a pull request with improvements or enhancements.

## License

Specify the license that applies to your project here. Common open-source licenses include MIT, GPL, or Apache 2.0.

---

