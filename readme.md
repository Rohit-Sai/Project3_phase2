Libraries used:

1) numpy
2) time
3) cv2
4) heapq
5) math

Teammembers:
Rohit Suresh, UID: 119283684, Directory ID: rohitsai@umd.edu
Sivaram Dheeraj Vishnubhotla, UID: 120427367

Githublink: https://github.com/Rohit-Sai/Project3_phase2


# A* Pathfinding Algorithm in ROS2

This README provides instructions on how to set up and run the A* pathfinding algorithm within a ROS2 environment, using the `a_star_ros_rohit_sivaram` package. Follow these steps to install and execute the package.

## Prerequisites

Ensure that ROS2 Galactic is installed on your system. You will also need a basic understanding of ROS2 concepts such as packages, nodes, and launches.

## Setup

1. **Source ROS2 Galactic**: Before starting, make sure your environment is set up to use ROS2 Galactic by sourcing its setup script:
    ```
    source /opt/ros/galactic/setup.zsh
    ```

2. **Create Workspace**: Navigate to your Desktop or a preferred workspace directory and create a new ROS2 workspace named `project3_ws`:
    ```
    cd ~/Desktop
    mkdir -p project3_ws/src
    ```

3. **Clone Package**: Place the `a_star_ros_rohit_sivaram` package inside the `src` directory of your newly created workspace.

4. **Build Package**: Navigate back to the root of your workspace and build the `a_star_ros_rohit_sivaram` package using `colcon`:
    ```
    cd ~/Desktop/project3_ws
    colcon build --packages-select a_star_ros_rohit_sivaram
    ```

## Execution

After setting up the workspace and building the package, follow these steps to run the A* pathfinding algorithm:

1. **Source the Workspace**: Ensure your shell session is aware of the newly built packages by sourcing the workspace:
    ```
    source ~/Desktop/project3_ws/install/setup.zsh
    ```

2. **Launch Simulation World** (Terminal 1):
    - Launch the competition world environment provided by the package:
        ```
        ros2 launch a_star_ros_rohit_sivaram competition_world.launch.py
        ```
      This will set up the simulation world where the A* pathfinding algorithm will be visualized.

3. **Run the A* Algorithm** (Terminal 2):
    - Run the main A* algorithm node:
        ```
        ros2 run a_star_ros_rohit_sivaram a_star_p2_2.py
        ```

    - When prompted, enter the following inputs:
        - **RPM of Wheels**: Input the RPM for the left and right wheels in the format `RPM1 RPM2`. For example, `10 15`.
        - **Clearance in mm**: Input the desired clearance in millimeters. For example, `20`. Note that the clearance will be considered as `24cm` in the algorithm.
        - **Start and End Nodes**: Enter the coordinates and orientation for the start node in the format `X Y Theta` (e.g., `0 0 0`), and the coordinates for the end node in the format `X Y` (e.g., `5.25 0`).

Drive Link for Project 3 phase 2: https://drive.google.com/drive/folders/13fKT7npcAF1ORCOQlLmDM8rgRnzXhmUu?usp=drive_link
