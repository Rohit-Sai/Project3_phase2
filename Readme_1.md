

# A* Path Planning with Obstacle Avoidance

This project is an implementation of the A* pathfinding algorithm tailored for a robot navigating through a predefined map with obstacles. It features obstacle avoidance by incorporating clearance and robot dimensions into the calculation, making it suitable for real-world applications, such as robotics and autonomous vehicles.

## Features

- **Obstacle Map Creation**: Generates a visual representation of the environment, including obstacles, free space, and robot dimensions.
- **Dynamic Path Planning**: Utilizes the A* algorithm to find the shortest path from a given start to an end point while avoiding obstacles.
- **Customizable Robot Parameters**: Allows configuration of robot dimensions, wheel RPMs, and clearance for flexible application to various robot models.
- **Visualization**: Provides a step-by-step visualization of the path planning process and outputs a final map showing the calculated path.
- **Video Output**: Saves a video of the path planning process, showcasing the algorithm's decision-making in real-time.

## Requirements

- Python 3.x
- NumPy
- OpenCV-Python

## Setup and Execution

1. **Installation**:

Ensure you have Python 3.x installed on your system. Install the required Python libraries using pip:

```sh
pip install numpy opencv-python
```

2. **Configuration**:

Before running the script, you can modify the start and end points, robot parameters (e.g., RPMs, radius), and obstacle configurations directly within the script.

3. **Running the Script**:

Execute the script with Python:

```sh
python a_star_path_planning.py
```

Follow the on-screen prompts to enter the RPMs of the wheels, clearance, start node, and end node. The coordinates for nodes are in meters, with the orientation given in degrees.

4. **Visualization and Output**:

After execution, a window will display the final path overlaid on the map. The script also saves a video of the path planning process in the working directory.

## Contributing

Contributions to improve the algorithm or extend its features are welcome. Please feel free to fork the repository, make your changes, and submit a pull request.




