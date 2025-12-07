# 3.2 Simulating Sensors (LiDAR & Depth) 👁️📡

Robots must “see” the world — and simulation must recreate these sensors accurately.
This chapter teaches how to simulate:

* 📡 LiDAR
* 📷 RGB-D Depth Camera
* 🧭 IMU
* 📦 Point clouds

All inside Gazebo.

---

# 🌟 Why Sensor Simulation?

Sensors allow robots to:

* Detect obstacles 🚧
* Map environments 🗺️
* Localize ⚪
* Navigate 🔄

Simulation lets you test everything without buying physical sensors.

---

# 🔍 1. Simulating LiDAR

LiDAR emits laser rays and measures return distances.

### LiDAR Capabilities:

* 360° scanning
* Distance detection
* Used in mapping (SLAM)
* Works in low light 🌙

---

## 📡 Example: Gazebo LiDAR Plugin

```xml
<!-- 📡 LiDAR Sensor Example -->
<sensor name="laser" type="ray">
  <pose>0 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.2</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <update_rate>20</update_rate>
  <topic>/scan</topic>
</sensor>
```

---

# 👁️ 2. Depth Camera Simulation

Depth cameras measure distance using infrared or stereo vision.

### Applications:

* Object detection 🎁
* Navigation
* 3D reconstruction
* Gesture tracking ✋

---

## 📷 Example: Depth Camera Plugin

```xml
<!-- 📷 Depth Camera -->
<sensor name="depth_camera" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_depth_camera.so">
    <camera_name>depth_cam</camera_name>
    <frame_name>depth_frame</frame_name>
    <depth_image_topic>/camera/depth</depth_image_topic>
    <point_cloud_topic>/camera/points</point_cloud_topic>
  </plugin>
</sensor>
```

---

# 🧭 3. IMU Sensor

IMU measures:

* Acceleration
* Rotation
* Angular velocity

---

## 📦 Example: IMU Plugin

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>50</update_rate>
  <topic>/imu</topic>
</sensor>
```

---

# 🧠 ROS 2 Integration

Once simulation publishes data:

| Topic            | Sensor       |
| ---------------- | ------------ |
| `/scan`          | LiDAR        |
| `/camera/depth`  | Depth Images |
| `/camera/points` | Point Clouds |
| `/imu`           | IMU          |

ROS 2 nodes can now process this data just like a real robot.

---

# 🎯 Summary

✔ LiDAR configured
✔ Depth camera simulated
✔ IMU added
✔ Topics ready for ROS 2
✔ Your robot can now “see” the world in simulation 👁️🤖
