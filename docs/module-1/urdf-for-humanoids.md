---
sidebar_position: 3
---

# Understanding URDF for Humanoids

A crucial aspect of robot simulation and control is having a standardized way to describe the robot's physical structure. This is where the Unified Robot Description Format (URDF) comes in. URDF is an XML-based file format used in ROS to describe all of the physical properties of a robot.

<details>
  <summary>Quick Summary</summary>
  <div>
    <p>
      This section explains the Unified Robot Description Format (URDF) and its importance for humanoid robots.
    </p>
    <ul>
      <li><strong>What is URDF?</strong> An XML-based format for describing a robot's physical properties, including links, joints, and visual appearance.</li>
      <li><strong>Key URDF Elements:</strong> Learn about the `<robot>`, `<link>`, and `<joint>` tags.</li>
      <li><strong>Humanoid Robots:</strong> Understand the complexity of modeling humanoids in URDF, with their many links and joints.</li>
      <li><strong>Visualization:</strong> See how URDF files are used by tools like RViz to visualize and debug robot models.</li>
    </ul>
  </div>
</details>

## Why is URDF Important?

A URDF file defines the robot's:

-   **Kinematics:** The arrangement of links and joints that make up the robot's body.
-   **Dynamics:** The mass, inertia, and other physical properties of each link.
-   **Visual Appearance:** The 3D models (meshes) that represent each part of the robot.
-   **Collision Properties:** The simplified shapes used for collision detection in a physics simulator.

This information is used by various ROS tools, including:

-   **State Publishers:** To broadcast the state of the robot's joints.
-   **Physics Simulators:** Like Gazebo, to simulate the robot's interaction with the world.
-   **3D Visualization Tools:** Like RViz, to display a 3D model of the robot.
-   **Motion Planning Libraries:** To plan paths for the robot's arms and legs.

## Core URDF Elements

A URDF file is structured around three core elements:

1.  **`<robot>`:** The root element of the file. It has a `name` attribute that defines the name of the robot.

2.  **`<link>`:** A link represents a rigid part of the robot's body. It has several child elements to define its properties:
    -   `<inertial>`: Defines the dynamic properties of the link (mass, inertia).
    -   `<visual>`: Defines the visual appearance of the link (geometry, material, origin). The geometry is often a 3D mesh file (e.g., in `.dae` or `.stl` format).
    -   `<collision>`: Defines the collision geometry of the link. This is often a simpler shape than the visual geometry to speed up collision checking.

3.  **`<joint>`:** A joint connects two links together and defines how they can move relative to each other. It has several important attributes and child elements:
    -   `name`: The name of the joint.
    -   `type`: The type of joint. Common types include:
        -   `revolute`: A hinge joint that rotates around a single axis (e.g., an elbow).
        -   `continuous`: A revolute joint with no angle limits.
        -   `prismatic`: A sliding joint that moves along a single axis.
        -   `fixed`: A joint that does not allow any motion.
    -   `<parent>` and `<child>`: These tags specify the two links that the joint connects.
    -   `<origin>`: Defines the transform from the parent link to the child link.
    -   `<axis>`: Defines the axis of rotation or translation for revolute and prismatic joints.
    -   `<limit>`: Defines the motion limits for the joint (e.g., upper and lower angle limits for a revolute joint).

## Modeling Humanoid Robots

Humanoid robots are significantly more complex to model in URDF than simple mobile robots. This is due to the large number of links (torso, head, arms, legs, hands) and joints required to replicate human-like motion.

A typical humanoid URDF will have:

-   A "base\_link" that serves as the root of the kinematic chain (often the torso or pelvis).
-   Kinematic chains for the arms, legs, and head, branching off from the base link.
-   Complex joint limits to ensure that the robot's movements are realistic and don't cause self-collisions.

## Example: A Simple Two-Link Arm

Here's a very simple example of a URDF for a two-link arm:

```xml
<robot name="simple_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_link_1">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_arm_1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link_1"/>
    <axis xyz="0 0 1"/>
    <limit effort="1000.0" lower="-3.14" upper="3.14" velocity="0.5"/>
  </joint>
</robot>
```

This URDF describes a simple arm with a cylindrical base and a rectangular first link. The `revolute` joint allows the arm to rotate around the Z-axis. In a real humanoid, you would have many more links and joints, all interconnected to form the complete robot.