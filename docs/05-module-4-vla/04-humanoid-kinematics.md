# 5.4 Humanoid Kinematics 🤖🦿

Humanoid robots are **bipedal machines** that require precise control for walking, balancing, and performing tasks.
This chapter dives into **forward and inverse kinematics**, **center of mass (CoM)** calculations, and **Jacobian matrices** for humanoid motion planning.

---

# 🌟 Why Humanoid Kinematics?

* Ensures **stable walking** without falling ⚖️
* Calculates **joint angles** to reach targets ✋
* Maintains **balance while interacting** with environment 🏋️
* Enables **precision tasks** like grasping or climbing 🛠️

---

# 🔍 1. Forward Kinematics (FK)

Forward kinematics calculates the **end-effector pose** (position and orientation) given **joint angles**.

### Example: 2-Joint Leg FK

```python
import numpy as np

# Link lengths
l1, l2 = 0.5, 0.5

# Joint angles (radians)
theta1 = np.pi/6
theta2 = np.pi/4

# FK equations
x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
y = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)

print(f"Foot position: x={x:.2f}, y={y:.2f}")
```

**Explanation**:

* θ1 = hip angle, θ2 = knee angle
* FK computes foot (end-effector) position from joint angles

---

# 🔍 2. Inverse Kinematics (IK)

Inverse kinematics computes **joint angles** to reach a desired foot or hand position.

```python
# Desired foot position
x_target, y_target = 0.7, 0.3

# Law of cosines
theta2 = np.arccos((x_target**2 + y_target**2 - l1**2 - l2**2)/(2*l1*l2))
theta1 = np.arctan2(y_target, x_target) - np.arctan2(l2*np.sin(theta2), l1+l2*np.cos(theta2))

print(f"Hip angle: {theta1:.2f}, Knee angle: {theta2:.2f}")
```

**Key Idea**: IK allows **precise placement** of feet or hands in space.

---

# 🧭 3. Center of Mass (CoM) & Balance

* CoM = weighted average of all link masses
* Maintaining CoM within **support polygon** ensures robot does not fall 🌟

```python
# Example: Simple 2-link CoM
m1, m2 = 10, 5  # kg
com_x = (m1*l1/2 + m2*(l1 + l2/2)) / (m1 + m2)
com_y = 0  # planar example

print(f"Center of Mass x={com_x:.2f}")
```

**Tip**: Always check **CoM projection** on the floor to maintain stability.

---

# 🧠 4. Jacobians & Velocity Mapping

Jacobian matrices relate **joint velocities** to **end-effector velocities**.
Essential for:

* Walking gait planning 🚶‍♂️
* Force control 🤲
* Dynamic balancing 🌀

```python
# Simple planar 2-link Jacobian
J = np.array([
    [-l1*np.sin(theta1) - l2*np.sin(theta1+theta2), -l2*np.sin(theta1+theta2)],
    [l1*np.cos(theta1) + l2*np.cos(theta1+theta2), l2*np.cos(theta1+theta2)]
])

# End-effector velocity
dq = np.array([0.1, 0.05])  # joint velocities
v = J @ dq
print(f"Foot velocity: vx={v[0]:.2f}, vy={v[1]:.2f}")
```

---

# 🎯 Summary

✔ Forward kinematics: compute positions from angles
✔ Inverse kinematics: compute angles from positions
✔ CoM ensures balance during walking or tasks
✔ Jacobians map joint motion to end-effector velocity
✔ Humanoid robots can now move safely, reach targets, and maintain dynamic stability 🤖🦿
