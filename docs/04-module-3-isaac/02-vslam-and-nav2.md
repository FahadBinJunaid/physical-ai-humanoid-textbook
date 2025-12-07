# 4.2 VSLAM & Nav2 📍🧭

VSLAM (Visual Simultaneous Localization and Mapping) allows robots to **map environments using cameras**.

Nav2 in ROS 2 enables **autonomous navigation** with path planning, obstacle avoidance, and goal-reaching.

---

# 🌟 Why VSLAM + Nav2?

* Robots can navigate **unknown environments** without pre-built maps 🗺️
* Cameras provide rich visual data 🌈
* Works in dynamic environments with moving obstacles 🚶‍♂️🚗

---

# 🔍 VSLAM Workflow

1. Capture images from RGB or RGB-D cameras 📷
2. Extract features (ORB, SIFT, SURF) ✨
3. Track feature points frame-to-frame 🔄
4. Estimate camera pose (position + orientation) 🧭
5. Build map incrementally while localizing

---

# 🛠️ Nav2 Overview

Nav2 provides:

* Global planner → computes path to goal 🗺️
* Local planner → avoids obstacles in real-time 🚧
* Recovery behaviors → handles robot getting stuck 🔄
* Lifecycle management → manages states of navigation nodes ⚡

---

# 🚀 Example: Launching Nav2 in ROS 2

```bash
# Launch Nav2 with TurtleBot3
ros2 launch nav2_bringup tb3_simulation_launch.py
```

---

# 🧠 Integrating VSLAM with Nav2

* VSLAM publishes `/odom` and `/map` topics
* Nav2 subscribes to `/map` and `/scan` for planning
* Robot can navigate using visual input and LiDAR combined

---

# 🎯 Summary

✔ VSLAM enables mapping with cameras
✔ Nav2 handles navigation and obstacle avoidance
✔ Integration allows fully autonomous robot movement in simulation and real world
