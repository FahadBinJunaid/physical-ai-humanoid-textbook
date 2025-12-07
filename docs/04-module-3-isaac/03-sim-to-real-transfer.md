# 4.3 Sim-to-Real Transfer 🔄🤖

Sim-to-Real transfer is the **process of deploying models trained in simulation onto real robots**.
It addresses the **reality gap** between simulated and physical environments.

---

# 🌟 Challenges

* Physics mismatch ⚖️
* Sensor noise & inaccuracies 📡
* Lighting and visual differences 🌅
* Robot wear-and-tear and mechanical tolerances ⚙️

---

# 🔍 Approaches to Minimize Reality Gap

1. **Domain Randomization**
   Randomize textures, lighting, and object positions in simulation 🎨

2. **Sensor Noise Modeling**
   Add realistic noise to LiDAR, cameras, and IMU measurements 📉

3. **Reinforcement Learning Fine-Tuning**
   Continue training with small real-world samples 🔧

4. **Calibration & Alignment**
   Ensure joint positions, wheel encoders, and camera intrinsics match the real robot 🛠️

---

# 🚀 Example: Domain Randomization in Isaac Sim

```python
# Randomly change environment lighting and object colors
import random
from omni.isaac.core.objects import DynamicCuboid

cube = DynamicCuboid(name="cube")
cube.set_color([random.random(), random.random(), random.random()])

# Randomize light intensity
world.get_light("sun").intensity = random.uniform(0.5, 2.0)
```

---

# 🧠 ROS 2 Integration for Real Robots

* Use ROS 2 topics from simulation as real robot topics
* Ensure TF frames match between sim and real hardware
* Use calibration scripts to align sensors and actuators

---

# 🎯 Summary

✔ Sim-to-Real is essential for deploying trained models
✔ Domain randomization reduces overfitting to simulation
✔ Sensor modeling and calibration improve real-world performance
✔ Proper integration allows seamless transfer from Isaac Sim to physical robots
