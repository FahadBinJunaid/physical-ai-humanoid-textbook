# 2.3 Defining the Body (URDF) 🦾📦

A robot is not just software — it is a **physical body** made of links, joints, sensors, and structure.  
To represent this body in simulation and ROS2, we use **URDF** (Unified Robot Description Format).

---

# 🧱 What is URDF?

URDF is an XML-based format used to describe:

- Robot size  
- Parts (links)  
- Connections (joints)  
- Sensors  
- Visual appearance  
- Collision shapes  
- Inertial properties  

It is the robot's **digital skeleton**.

---

# 🔩 URDF Structure

A robot consists of:

### 🔹 Links  
Rigid parts (body, wheels, arm segments).

### 🔹 Joints  
Connections between links:
- Revolute (rotate)  
- Continuous  
- Fixed  
- Prismatic (slide)  

---

# 🦿 Example: A Simple Two-Link Robot Arm

base_link ─── joint1 ─── link1 ─── joint2 ─── link2

php-template
Copy code

URDF describes this entire structure.

---

# 🎨 Visual + Collision Models

Each link can have:

- **Visual**: How it looks  
- **Collision**: How physics interacts  
- **Inertial**: How heavy it is  

These are essential for real simulation.

---

# 📘 URDF Code Example (Copy-Ready)

```xml
<!-- 🤖 Simple Two-Link URDF Example -->

<robot name="two_link_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.4 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>