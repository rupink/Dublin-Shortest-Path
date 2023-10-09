# Path Planning with RRT and Dubins Paths

This repository provides a Python implementation of path planning using the RRT (Rapidly-exploring Random Tree) algorithm with Dubins paths. This algorithm is designed for planning paths for vehicles with Dubins dynamics, making it suitable for various mobile robots and vehicles. The code allows you to plan a path from a start location to a goal location while avoiding obstacles.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Usage](#usage)
- [Installation](#installation)
- [Configuration](#configuration)
- [Running the Code](#running-the-code)
- [Visualization](#visualization)
- [Example](#example)
- [Contributing](#contributing)
- [License](#license)

## Overview

Path planning is a fundamental problem in robotics and autonomous navigation. It involves finding a collision-free path from a starting point to a desired goal while avoiding obstacles. The RRT algorithm is a popular choice for solving this problem efficiently. In this repository, we extend the RRT algorithm to handle Dubins paths, which are paths that connect two points with a minimum turning radius.

## Features

- **RRT-based Path Planning**: The code implements path planning using the RRT algorithm, which explores the configuration space to find a feasible path.

- **Dubins Paths**: The algorithm is designed to work with Dubins paths, which are suitable for vehicles with constrained turning radii.

- **Obstacle Avoidance**: The planner ensures that the generated path avoids obstacles, represented as circular obstacles in the workspace.

- **Customizable Start and Goal**: You can set the start and goal locations according to your application's requirements.

- **Visualization**: The code provides visualization capabilities to help you visualize the planned path and obstacles.

## Prerequisites

Before using this code, ensure that you have the following prerequisites installed:

- Python 3.x
- NumPy
- Matplotlib

## Usage

Follow these steps to use the path planning code:

### Installation

Clone this repository to your local machine:

```bash
git clone https://github.com/yourusername/path-planning-rrt-dubins.git
```

### Configuration

Configure the start and goal locations, as well as the obstacle positions, in the code to match your specific scenario.

### Running the Code

Execute the main script to run the path planning algorithm:

```bash
python main.py
```

### Visualization

The code provides visualization capabilities to help you understand the path planning process and visualize the results.

## Example

Here's an example of how to use the path planning code:

```python
# Set the start and goal positions
start = [0.0, 0.0, 0.0]
goal = [10.0, 10.0, -45.0]

# Define circular obstacles
obstacle_list = [
    (5, 5, 1),
    (3, 6, 2),
    (3, 8, 2),
    (3, 10, 2),
    (7, 5, 2),
    (9, 5, 2)
]

# Create an instance of the path planner
rrt_dubins = RRTDubinsPlanner(start=start, goal=goal, obstacle_list=obstacle_list)

# Plan the path
path = rrt_dubins.plan_path()

# Visualize the path and obstacles
rrt_dubins.visualize_path()
```

## Contributing

Contributions to this project are welcome. You can contribute by reporting issues, suggesting enhancements, or submitting pull requests. For more information, please refer to the [Contribution Guidelines](CONTRIBUTING.md).

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
