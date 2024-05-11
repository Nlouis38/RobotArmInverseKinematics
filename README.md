# Pinocchio Robot Arm Inverse Kinematics (IK) with Joint Limits

This project demonstrates a robot arm inverse kinematics solver using the Pinocchio library and CVXPY for joint limit enforcement. It allows you to interactively specify desired end-effector positions, and the solver computes joint angles that bring the end-effector as close as possible to the target while respecting joint limits.

## How It Works

1. **Robot Model:** The code loads the robot model from a URDF (Unified Robot Description Format) file. This file defines the robot's structure, links, joints, and joint limits.

2. **Desired Pose:** The user provides the desired x, y, and z coordinates of the end-effector position.

3. **Iterative IK Solver:** The solver iteratively updates the joint angles to minimize the error between the current end-effector position and the desired position. It uses a Jacobian-based method, where the Jacobian matrix relates changes in joint angles to changes in the end-effector pose.

4. **Joint Limits:** To ensure physically realistic solutions, the solver incorporates joint limits as constraints. It uses a Quadratic Program (QP) solver (CVXPY with OSQP) to find the optimal joint angles that are within the allowed range.

5. **Visualization:** If the solver finds a valid solution, the robot arm is visualized in its final configuration using Matplotlib.

## Installation

1. **Prerequisites:** Ensure you have Python (3.6 or later) and pip (Python package installer) installed on your system.

2. **Create a Virtual Environment (Recommended):**
   ```bash
   python -m venv robot_ik_env 
