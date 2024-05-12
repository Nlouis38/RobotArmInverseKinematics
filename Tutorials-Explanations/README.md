1.	Imports and Initialization:
```
from __future__ import print_function
import numpy as np
from numpy.linalg import norm, solve
import pinocchio
import cvxpy as cp
import time
import matplotlib.pyplot as plt

URDF_FILENAME = r"C:\Users\nazir\OneDrive\Documents\Python Programs\RobotArm_Pin\robotArm.urdf" 
model = pinocchio.buildModelFromUrdf(URDF_FILENAME)
data = model.createData()
```
•	Import necessary modules: Imports the required Python libraries for calculations, optimization, and visualization.
•	Load robot model from URDF: 
o	URDF_FILENAME: Set to the path of your robot's URDF file (replace with the actual path).
o	model: Loads the robot model description from the URDF file using Pinocchio.
o	data: Creates a data structure to store the robot's state during calculations.
2. Configuration and Parameters:
```
JOINT_ID = 6 
q = pinocchio.neutral(model) 
eps = 0.025 
IT_MAX = 500 
DT = 1e-1  
damp = 1e-12
```
•	JOINT_ID: Defines the index of the end effector joint (the one you want to control). Typically this is the last joint in the robot arm's kinematic chain.
•	q: Initializes a vector q with the robot's initial joint angles. The pinocchio.neutral(model) function sets all angles to a "default" or "home" position.
•	eps: Sets the tolerance for the position error. The IK algorithm will stop when the distance between the desired and actual end-effector positions is less than this value.
•	IT_MAX: Defines the maximum number of iterations for the IK algorithm. If the desired position isn't reached within this limit, the algorithm will stop and indicate non-convergence.
•	DT: The integration time step used in the IK algorithm. A smaller value generally leads to higher accuracy but might increase computation time.
•	damp: A small damping factor added to the Jacobian matrix. This is a regularization technique to prevent numerical instabilities that can arise during calculations, especially when the Jacobian is ill-conditioned (close to singular).
2.	Joint Limits
```
lower_limits = model.lowerPositionLimit
upper_limits = model.upperPositionLimit
```
•	Reads the lower and upper joint limits from the URDF model. These limits will be enforced as constraints in the optimization process.
4. Main Loop:
```
while True:
    input_str = input("Enter desired end-effector position (x y z, or type 'q' to quit): ")
    if input_str.lower() == 'q':
        break

    try:
        x, y, z = map(float, input_str.split())
    except ValueError:
        print("Invalid input format. Please enter 3 numbers separated by spaces.")
        continue
    oMdes = pinocchio.SE3(np.eye(3), np.array([x, y, z]))
    start_time = time.time()
```
•	Purpose: Continuously prompts the user for target end-effector positions until they quit.
•	Key Points: 
o	Takes user input in the format "x y z".
o	Handles invalid input with a try-except block.
o	oMdes represents the desired pose (position and orientation) of the end-effector.
o	start_time variable records the current time before starting the IK iterations to measure computation time.
5. Iterative IK Algorithm:
```
    i = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[JOINT_ID].actInv(oMdes)
        err = pinocchio.log(iMd).vector
        
        pos_err = err[:3]

        if norm(pos_err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        
        # ... rest of the code (Jacobian, optimization, etc.) ...
```
•	Purpose: This is the core of the IK solver. It iteratively adjusts the joint angles to move the end-effector towards the desired position.
•	pinocchio.forwardKinematics(model, data, q): 
o	Performs forward kinematics, calculating the position of the end-effector given the current joint angles q.
•	iMd = data.oMi[JOINT_ID].actInv(oMdes): 
o	Calculates the difference between the current end-effector pose and the desired pose.
•	err = pinocchio.log(iMd).vector: 
o	Converts the pose difference into a 6D "twist" vector representing the positional and orientation errors.
•	pos_err = err[:3]: 
o	Extracts the positional error (x, y, z) from the err vector.
•	Convergence Checks: 
o	if norm(pos_err) < eps: If the magnitude of the positional error is less than the tolerance (eps), the algorithm has converged and exits the loop.
o	if i >= IT_MAX: If the number of iterations reaches the maximum limit (IT_MAX), the algorithm has not converged and exits the loop.
6. Jacobian Calculation and Transformation:
Python
        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID) 
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
Use code with caution.
play_circleeditcontent_copy
•	J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID): 
o	Calculates the Jacobian matrix for the end-effector joint. The Jacobian relates joint velocities to end-effector velocities in the robot's current configuration.
•	J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J): 
o	Transforms the Jacobian from joint space to task space (Cartesian space). This is necessary because we want to control the end-effector's position directly.
•	7. QP Optimization (Calculate Joint Angle Updates):
```
        q_next = cp.Variable(model.nq)  

        desired_q_change = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err)) * DT
        objective = cp.Minimize(cp.sum_squares(q_next - q - desired_q_change))

        constraints = [lower_limits <= q_next, q_next <= upper_limits]

        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False) 

        q = q_next.value 
        if i % 10 == 0: 
            print(f"{i}: error = {err.T}") 
            print(f"result: {q.flatten().tolist()}")
```
•  Purpose: Calculates the optimal joint angle updates using quadratic programming (QP) optimization, taking joint limits into account. 
•  q_next = cp.Variable(model.nq): Creates a variable representing the next set of joint angles, which will be optimized. 
•  desired_q_change: Calculates the desired change in joint angles using a damped least-squares method. This aims to minimize the error between the current and desired end-effector poses. 
•  objective: Defines the optimization objective: to minimize the difference between the desired joint angle change (desired_q_change) and the actual change (q_next - q). 
•  constraints: Sets constraints to ensure the joint angles stay within their physical limits. 
•  prob = cp.Problem(objective, constraints): Formulates the QP problem with the defined objective and constraints. 
•  prob.solve(...): Solves the QP problem using the OSQP solver (a fast and robust solver for QPs). The warm_start=True option can help speed up convergence. 
•  q = q_next.value: Updates the joint angles q with the optimal values found by the solver. 
•  Progress Reporting: The if i % 10 == 0 block prints the current iteration number and the error every 10 iterations, providing feedback on the solver's progress.



