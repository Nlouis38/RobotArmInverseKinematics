# Creating URDF Files for the Robot Arm IK Solver

This document provides instructions on how to create a URDF (Unified Robot Description Format) file that is compatible with our robot arm inverse kinematics (IK) solver.

## What is a URDF?

A URDF file is an XML file that describes the physical properties of a robot, including the links (rigid bodies), joints (connections between links), and associated visual and collision geometries. This information is crucial for the IK solver to accurately calculate joint angles that achieve desired end-effector positions.

## URDF Requirements for the IK Solver

To ensure compatibility with the IK solver, your URDF file must adhere to the following guidelines:

1. **Accurate Dimensions:**
   - **Link lengths:**  Ensure the lengths of each link (e.g., shoulder, bicep, forearm) match the physical dimensions of your robot arm.
   - **Joint positions:**  Specify the correct position of each joint relative to its parent link. This is typically defined using the `<origin>` tag within the `<joint>` element.
   - **Axis orientations:**  Use the `<axis>` tag to define the axis of rotation for each joint. Ensure these axes align with the actual joint movements of your robot arm.

2. **Joint Types and Limits:**
   - **Joint types:** Use the appropriate joint type (`revolute`, `prismatic`, etc.) for each joint in your robot arm. The IK solver relies on this information to determine valid joint movements.
   - **Joint limits:**  Specify the minimum (`lower`) and maximum (`upper`) limits for each joint's range of motion. This prevents the IK solver from attempting to move joints beyond their physical capabilities.

3. **Collision Geometry:**
   - **Collision shapes:** Define collision shapes (e.g., boxes, cylinders) that approximate the physical shape of each link. This is important for collision avoidance in more advanced IK solvers.

4. **Complete Kinematic Chain:**
   - **All links and joints:** Include all the links and joints that make up the kinematic chain of your robot arm, from the base link to the end effector.
   - **Base link:** Ensure the first link is named "base_link" and is fixed to the world frame. This establishes the reference for all other transformations.

## Example URDF

Refer to the provided example URDF file for a sample implementation of these guidelines. You can use this as a template and modify it to match the specific structure of your robot arm.
