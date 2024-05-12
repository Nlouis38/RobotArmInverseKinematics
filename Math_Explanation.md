# Mathematical Foundation of the Robot Arm Inverse Kinematics (IK) Solver

This document provides a deeper dive into the mathematical concepts and formulas used in the provided IK solver, specifically focusing on Jacobian matrices and the OSQP (Operator Splitting Quadratic Program) solver.

## Jacobian Matrices

### What is a Jacobian?

In robotics, the Jacobian matrix, often denoted as **J**, is a matrix of partial derivatives that describes the relationship between joint velocities and end-effector velocities. It provides a linear approximation of how small changes in joint angles affect the position and orientation of the robot's end effector.

### Jacobian Calculation (General Form)

The Jacobian matrix for a robot manipulator with *n* joints is typically a 6 x *n* matrix, where each column represents the effect of one joint on the end-effector velocity.  The general form is:

Where:

- `J_v` is the linear velocity Jacobian (3 x *n*)
- `J_w` is the angular velocity Jacobian (3 x *n*)

Each element `J(i,j)` of the Jacobian represents the partial derivative of the *i*th component of the end-effector velocity with respect to the *j*th joint velocity.

### Jacobian in IK

In inverse kinematics, we want to find the joint velocities (`dq`) that will produce a desired end-effector velocity (`dx`).  We can use the Jacobian to approximate this relationship:

To solve for `dq`, we need to invert the Jacobian. However, the Jacobian is often not square and may not be directly invertible. In these cases, we use the pseudo-inverse of the Jacobian:

Where `J^+` is the pseudo-inverse of the Jacobian. In the provided code, this is calculated indirectly using the `pinocchio.Jlog6` and matrix inversion operations.

## OSQP (Operator Splitting Quadratic Program) Solver

### What is OSQP?

OSQP is a numerical optimization solver specifically designed for solving quadratic programs (QPs) with the following form:

where:

- `x` is the vector of optimization variables
- `P` is a positive semidefinite matrix
- `q` is a vector
- `A` is a matrix
- `l` and `u` are vectors representing lower and upper bounds

### How is it Used in IK?

The IK solver formulates the problem of finding the optimal joint velocities as a QP:

- **Objective Function:** Minimize the norm of the error between the current and desired end-effector velocity.
- **Constraints:** Ensure that the joint limits are not violated.

OSQP efficiently solves this QP, providing the optimal joint velocities that move the robot arm closer to the desired position while respecting its physical constraints.

### OSQP in the Code

In the code, the `cvxpy` library is used to define the QP problem in a user-friendly way. The `prob.solve(solver=cp.OSQP)` line calls the OSQP solver to find the optimal solution.

## Key Points

- The Jacobian matrix is a fundamental tool in robotics, providing a linear approximation of the relationship between joint and end-effector velocities.
- The IK solver uses the Jacobian (or its pseudo-inverse) to iteratively update joint angles to reach a desired end-effector position.
- OSQP is a powerful optimization solver that efficiently handles the constrained QP formulation of the IK problem.
