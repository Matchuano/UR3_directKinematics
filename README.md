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


## Prerequisites

### Software Requirements

- **CoppeliaSim**: Ensure that you have CoppeliaSim installed and that the remote API is enabled.
- **Python 3.x**: The script is written in Python and requires Python 3.x to run.

### Python Dependencies

- **NumPy**: Used for mathematical computations.
- **Robotics Toolbox for Python**: Required for handling inverse kinematics.
- **SpatialMath**: Required for managing 3D spatial transformations.

## Running the Simulation

1. Launch CoppeliaSim and open the `cenarioUR3.ttt` scene.
2. Run the CoppeliaSim scene.
3. Run the Python script to control the UR3 robot:
   ```bash
   python directKinematics.py
4. The script will move the robot through several joint configurations and print the current joint and end-effector positions in the terminal.

## Example Usage

Once the simulation is running, the script provides a variety of functionalities:

- **Set joint positions:** Moves the UR3 robot joints to specified angles (in radians).
  ```python
  ur3.set_joint_position([0, 0, 0, 0, 0, 0])
- **Retrieve current joint positions:**
  ```python
  print(ur3.get_joint_position())
- **Calculate the end-effector position based on the current joint angles:**
  ```python
  print(ur3.calculate_effector_position())
- **Inverse kinematics to move the end-effector to a target position:**
  ```python
  ur3.inverse_kinematics([0.1, 0.2, 0.5])

## Authors

- Matheus de Sousa Luiz
- Nathan Spinola Zeidan
