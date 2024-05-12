# Robot Arm Inverse Kinematics (IK) with Joint Limits

This project demonstrates a robot arm inverse kinematics (IK) solver using the Pinocchio library and CVXPY for joint limit enforcement. It allows you to interactively specify desired end-effector positions, and the solver computes joint angles that bring the end-effector as close as possible to the target while respecting joint limits.

## How It Works

1. **Robot Model:**
   - Loads the robot's kinematic structure from a URDF file. This file defines the robot's links, joints, and joint limits.
   - Uses Pinocchio's `buildModelFromUrdf` function to create a model object (`model`).
   - Creates a data structure (`data`) to store kinematic information during computations.

2. **Task Setup:**
   - Specifies the joint ID (`JOINT_ID`) of the end-effector you want to control. 

3. **IK Solver Parameters:**
   - Defines parameters for the IK solver:
      - `eps`: Positional error tolerance (in meters).
      - `IT_MAX`: Maximum number of iterations for the solver.
      - `DT`: Integration time step for joint updates.
      - `damp`: Damping factor for numerical stability in the pseudo-inverse Jacobian calculation.
   - Sets up parameters for the OSQP solver (used by CVXPY for the QP optimization):
      - `OSQP_SETTINGS`: Dictionary containing OSQP parameters like `max_iter`, `eps_abs`, `eps_rel`, and `polish`.

4. **Joint Limit Setup:**
   - Retrieves the lower and upper joint limits for all joints from the model.

5. **Main Loop (Input and IK Solver):**
   - Continuously prompts the user for the desired end-effector position (x, y, z).
   - If the user enters 'q', the loop terminates.
   - Parses the user's input into x, y, and z coordinates.
   - Creates a `pinocchio.SE3` object (`oMdes`) representing the desired pose (position and orientation).
   - Starts a timer to measure the time taken to solve the IK problem.
   - Initializes the joint angles (`q`) to the robot's neutral configuration for each new target.
   - **Iterative IK Solver:**
      - Enters a loop that iterates until convergence or the maximum number of iterations is reached.
      - Performs forward kinematics to get the current end-effector pose.
      - Calculates the error between the current and desired pose.
      - Checks if the positional error is within the tolerance (`eps`). If so, marks the solution as successful and breaks the loop.
      - Computes the Jacobian matrix, which relates changes in joint angles to changes in the end-effector pose.
      - Uses CVXPY to formulate and solve a Quadratic Program (QP) to find the next set of joint angles. The QP objective is to minimize the deviation from the desired change in joint angles, and the constraints enforce the joint limits.
      - Updates the joint angles with the solution from the QP solver.
      - Prints the error and current joint angles every 10 iterations for debugging purposes.

6. **Result Output and Visualization:**
   - If a valid solution is found (within tolerance):
     - Prints "Convergence achieved!"
     - Prints the time it took to solve the IK problem.
     - Prints the final joint angles in degrees.
     - Visualizes the robot arm in its final configuration using Matplotlib.
   - If no valid solution is found within the specified number of iterations, prints a warning message.

# Installation Instructions

This project requires several Python libraries to be installed. You can use either `pip` or `conda` as your package manager.

## Prerequisites

- **Python:** Make sure you have Python 3.6 or later installed on your system. You can check your Python version by typing `python --version` in your terminal.
- **pip (or conda):** You'll need either `pip` (Python's package installer) or `conda` (a cross-platform package and environment manager) to install the libraries.

## Option 1: Installation with `pip`

1. **Create a Virtual Environment (Recommended):**
   - It's good practice to create a virtual environment to isolate the dependencies of this project from your other Python projects.
   - Open your terminal and navigate to your project directory.
   - Create the environment:
     ```bash
     python -m venv robot_ik_env 
     ```

2. **Activate the Environment:**
   - **Windows:** `robot_ik_env\Scripts\activate`
   - **macOS/Linux:** `source robot_ik_env/bin/activate`

3. **Install Libraries:**
   - Install all the required libraries with the following command:
     ```bash
     pip install pinocchio numpy matplotlib cvxpy osqp
     ```

## Option 2: Installation with `conda`

1. **Create a Conda Environment (Recommended):**
   - If you prefer using conda, create a new environment for this project:
     ```bash
     conda create -n robot_ik_env python=3.8  # Or your desired Python version
     ```

2. **Activate the Environment:**
   ```bash
   conda activate robot_ik_env
