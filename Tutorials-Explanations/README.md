# Inverse Kinematics (IK) Solver for Robot Arm with Pinocchio and CVXPY

# --- Imports and Compatibility ---

from __future__ import print_function  # Ensure compatibility with Python 2's print function
import numpy as np  # Numerical operations library
from numpy.linalg import norm, solve  # Import functions for vector norms and linear equations
import pinocchio  # Library for rigid body dynamics modeling and kinematics
import cvxpy as cp  # Convex optimization library
import time         # Measure execution time
import matplotlib.pyplot as plt  # For optional 3D visualization

# --- Load Robot Model ---

URDF_FILENAME = r"C:\...\robotArm.urdf"  # Path to your robot's URDF file (update this!)
model = pinocchio.buildModelFromUrdf(URDF_FILENAME)  # Construct the robot model
data = model.createData()  # Create data structure to store robot's state during calculations

# --- IK Target Joint ---

JOINT_ID = 6  # Index of the joint you want to control (usually the end effector)

# --- IK Algorithm Parameters ---

q = pinocchio.neutral(model)  # Initial joint configuration (all zeros)
eps = 0.025  # Error tolerance: How close to get to the target (meters)
IT_MAX = 500  # Maximum iterations before giving up
DT = 1e-1   # Integration time step (affects convergence speed and stability)
damp = 1e-12 # Damping factor for numerical stability (prevents singular matrices)

# --- Joint Limits (Read from URDF) ---

lower_limits = model.lowerPositionLimit
upper_limits = model.upperPositionLimit

# --- Main Loop (Keep Asking for Targets) ---

while True:
    # --- Get User Input ---
    input_str = input("Enter desired end-effector position (x y z, or type 'q' to quit): ")
    if input_str.lower() == 'q':
        break  # Exit the loop if the user enters 'q'

    try:
        x, y, z = map(float, input_str.split())  # Convert input string to float values
    except ValueError:
        print("Invalid input. Please enter 3 numbers separated by spaces.")
        continue  # Skip to the next iteration of the loop if input is invalid

    # --- Set IK Target ---
    oMdes = pinocchio.SE3(np.eye(3), np.array([x, y, z]))  # Create a transformation matrix representing the desired pose

    # --- Start Timing ---
    start_time = time.time()

    # --- Iterative IK Algorithm ---
    for i in range(IT_MAX):
        # --- Forward Kinematics ---
        pinocchio.forwardKinematics(model, data, q)  # Calculate current end effector position
        
        # --- Calculate Error ---
        iMd = data.oMi[JOINT_ID].actInv(oMdes)  # Difference between current and desired pose
        err = pinocchio.log(iMd).vector  # Logarithm of the error, giving a 6D "twist" (dx, dy, dz, rotation errors)
        pos_err = err[:3]  # Extract only the positional error (x, y, z)

        # --- Check for Convergence ---
        if norm(pos_err) < eps:
            success = True
            break  # Exit loop if close enough to the target

        # --- Maximum Iterations Reached ---
        if i >= IT_MAX:  # Exit loop if we've iterated too many times
            success = False
            break

        # --- Compute Jacobian ---
        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # Jacobian for the end-effector joint
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)  # Jacobian in the task space (6D)

        # --- Solve for Joint Updates (QP Optimization) ---
        q_next = cp.Variable(model.nq)  # Create optimization variables (joint angles)
        
        # Objective: Minimize deviation from desired joint angle change using damped least squares
        desired_q_change = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err)) * DT 
        objective = cp.Minimize(cp.sum_squares(q_next - q - desired_q_change))

        # Constraints: Keep joint angles within limits
        constraints = [lower_limits <= q_next, q_next <= upper_limits]

        # Solve the optimization problem to find the best joint angle updates
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False) 

        # Update the joint angles
        q = q_next.value

        # Print progress
        if i % 10 == 0: 
            print('%d: error = %s' % (i, err.T))
            print('\nresult: %s' % q.flatten().tolist())

    # --- Output Results ---
    if success:
        end_time = time.time()
        solve_time = end_time - start_time

        print("Convergence achieved!")
        print(f"\nTime to solve: {solve_time:.4f} seconds")
        print(f"\nresult (degrees): {np.degrees(q.flatten()).tolist()}")  # Print angles in degrees

        # --- 3D Visualization (Optional) ---
        joint_positions = [data.oMi[j].translation for j in range(model.njoints)]  # Get joint positions
        endpoint = data.oMi[JOINT_ID].translation.copy()                          # Get end effector position

        if 'visualize_robot_arm' in locals():  # If the visualization function exists
            visualize_robot_arm(joint_positions, endpoint)  # Visualize the robot arm
    else:
        print("\nWarning: the iterative algorithm did not converge.")
