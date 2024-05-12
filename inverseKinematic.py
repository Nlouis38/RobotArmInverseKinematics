from __future__ import print_function  # Compatibility handling for print function
import numpy as np 
from numpy.linalg import norm, solve   # NumPy for array operations and linear algebra
import pinocchio  # Import the Pinocchio library
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import cvxpy as cp
import time  # Import the time module

# -- Model Setup --
URDF_FILENAME = r"C:\Users\nazir\OneDrive\Documents\Python Programs\RobotArm_Pin\robotArm.urdf"  # Replace with the path to your URDF file
model = pinocchio.buildModelFromUrdf(URDF_FILENAME)  # Build a standard manipulator robot model
data = model.createData()  # Create a structure to store robot dynamics data

# -- Task Setup --
JOINT_ID = 6  # ID of the joint to control (assuming a 6+ joint manipulator)

# -- Initialize Configuration --
q = pinocchio.neutral(model)  # Get a neutral/reference starting configuration 
eps = 0.025 # Tolerance for error
IT_MAX = 500  # Maximum iterations
DT = 1e-1  # Time step for simulation-like iteration
damp = 1e-12  # Damping term for numerical stability

# --- Joint Limit Setup ---
lower_limits = model.lowerPositionLimit   
upper_limits = model.upperPositionLimit

while True:
    # Get user input for desired position (x, y, z)
    input_str = input("Enter desired end-effector position (x y z, or type 'q' to quit): ")
    if input_str.lower() == 'q':
        break  # Quit the loop

    try:
        x, y, z = map(float, input_str.split())
    except ValueError:
        print("Invalid input format. Please enter 3 numbers separated by spaces.")
        continue
    # --- Task Setup (Update oMdes based on input) ---
    oMdes = pinocchio.SE3(np.eye(3), np.array([x, y, z]))

    # Start the timer
    start_time = time.time()

    # -- IK Iterative Loop --
    i = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)  # Compute position of all joints
        iMd = data.oMi[JOINT_ID].actInv(oMdes)  # Error between current and desired pose
        err = pinocchio.log(iMd).vector  # Express error in joint frame 
        
        # Extract positional error
        pos_err = err[:3]  # Get the first 3 elements of err (x, y, z errors)

        if norm(pos_err) < eps:  # Check if error is below tolerance
            success = True
            break  # Exit the loop if success
        if i >= IT_MAX: 
            success = False
            break  # Exit if maximum iterations are reached

        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # Joint Jacobian
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)  # Transform Jacobian

        # --- CVXPY QP Optimization ---
        q_next = cp.Variable(model.nq)  # Joint angles

        # Objective: Minimize deviation from desired change in joint angles
        desired_q_change = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err)) * DT
        objective = cp.Minimize(cp.sum_squares(q_next - q - desired_q_change))

        # Constraints: Enforce joint limits for all joints
        constraints = [lower_limits <= q_next, q_next <= upper_limits]

        # Create and solve the QP problem
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP,warm_start=True,verbose=False)

        # Update joint configuration
        q = q_next.value

        if not i % 10:  # Print error every 10 iterations
            print('%d: error = %s' % (i, err.T))
            print('\nresult: %s' % q.flatten().tolist()) 
        i += 1

    # -- Result Output --
    if success:
        end_time = time.time()  # Stop the timer
        solve_time = end_time - start_time  # Calculate solve time

        print("Convergence achieved!")
        print(f"\nTime to solve: {solve_time:.4f} seconds")  # Print solve time
        print('\nresult (degrees): %s' % np.degrees(q.flatten()).tolist())  # Convert to degrees

        # ------ Visualization --------
        def visualize_robot_arm(joint_positions, endpoint):
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            x_values = [point[0] for point in joint_positions]
            y_values = [point[1] for point in joint_positions]
            z_values = [point[2] for point in joint_positions]

            ax.plot(x_values, y_values, z_values, 'b-') 
            ax.scatter(endpoint[0], endpoint[1], endpoint[2], c='r', marker='o')

            ax.set_xlabel('X') 
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_xlim([-0.5, 0.5])  # Adjust limits if needed 
            ax.set_ylim([-0.5, 0.5])   
            ax.set_zlim([-0.5, 0.5])   

            plt.show()

        # Extract joint positions
        joint_positions = [data.oMi[joint_id].translation for joint_id in range(model.njoints)] 
        endpoint = data.oMi[JOINT_ID].translation.copy() 
        visualize_robot_arm(joint_positions, endpoint)

    else:
        print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
