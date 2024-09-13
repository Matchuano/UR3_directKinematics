# UR3 Robotic Arm Control in CoppeliaSim

This repository contains the final project for the **Fundamentals of Robotics** course, taught by Professor **Roberto Baptista** at the University of Brasília (UnB), Gama campus (FGA), during the first semester of 2024. The project was developed by **Matheus de Sousa Luiz** and **Nathan Spinola Zeidan** on **September 12, 2024**.

## Project Overview

The goal of this project is to control the **UR3 robotic arm** using the **CoppeliaSim** simulation environment. This project demonstrates key robotics concepts, including joint control, forward and inverse kinematics, and spatial transformations.

The project includes the necessary code and simulation files to interact with the robot in a simulated environment. The robotic arm can be controlled programmatically to move through various joint configurations, calculate its end-effector position, and solve for inverse kinematics to reach a specified point.

## Features

- **CoppeliaSim Integration**: Full control of the UR3 robot via the CoppeliaSim remote API.
- **Joint Control**: Set and retrieve the joint positions of the UR3 robot.
- **Forward Kinematics**: Compute the position of the end-effector based on joint angles using Denavit-Hartenberg parameters.
- **Inverse Kinematics**: Calculate the joint angles required to move the end-effector to a desired position.
- **Real-Time Simulation**: Perform real-time robot control and observe the results in the CoppeliaSim environment.

## Repository Contents

- `ur3_control.py`: Main Python script that connects to CoppeliaSim and provides functions for controlling the UR3 robotic arm.
- `simulation/`: Directory containing CoppeliaSim scene files and models.
  - `ur3_simulation.ttt`: CoppeliaSim scene for the UR3 robot.
- `requirements.txt`: List of dependencies required for running the Python script.
- `README.md`: This documentation file with setup and usage instructions.

## Prerequisites

### Software Requirements

- **CoppeliaSim**: Ensure that you have CoppeliaSim installed and that the remote API is enabled.
- **Python 3.x**: The script is written in Python and requires Python 3.x to run.
- **Python Libraries**: The project requires several Python libraries that can be installed using the provided `requirements.txt` file.

### Python Dependencies

- **NumPy**: Used for mathematical computations.
- **Robotics Toolbox for Python**: Required for handling inverse kinematics.
- **SpatialMath**: Required for managing 3D spatial transformations.

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/ur3-robot-control.git
   cd ur3-robot-control

