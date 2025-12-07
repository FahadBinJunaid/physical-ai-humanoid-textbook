# 3.3 High-Fidelity Visualization (Unity) 🎮✨

Gazebo is great for physics — but Unity gives you **cinematic visuals**, **UI**, and **interactive dashboards**.

Unity becomes your robot visualization engine with:

* Realistic lighting
* HD surface materials
* Post-processing
* Smooth camera tracking
* ROS communication

---

# 🌟 Why Use Unity for Robots?

Unity helps you:

* Build beautiful demos 🎥
* Visualize robot paths clearly
* Create VR/AR interfaces
* Test UI dashboards
* Make training environments

Unity + ROS makes simulation look REAL.

---

# 🔌 ROS–Unity Communication

Unity communicates using:

### ✔️ ROS-TCP-Connector

Unity → ROS messages (Twist, Pose, Image)

### ✔️ ROS-TCP-Endpoint

ROS → Unity messages
Runs as a ROS 2 Python package.

---

# 🛠️ Setup Overview

1. Install Unity (2022+ recommended)
2. Add ROS-TCP-Connector package
3. Run ROS-TCP-Endpoint in ROS 2
4. Configure message types
5. Create Unity scene with robot model

---

# 🎮 Features You Can Build

### 🌈 1) Real-Time Path Visualization

Show robot movement with glowing trails.

### 📺 2) Live Camera Feeds

Unity displays ROS image topics.

### 🎛️ 3) Control Panels

Buttons, sliders → control robot velocity.

### 🧱 4) High-Fidelity Floors & Environments

Warehouse, home, street, lab scenes.

---

# 📡 Example: Sending Velocity Command From Unity

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnityToRosCmd : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("cmd_vel");
    }

    void Update()
    {
        // 🕹️ Move forward when pressing W
        if (Input.GetKey(KeyCode.W))
        {
            TwistMsg msg = new TwistMsg();
            msg.linear.x = 0.5f;
            msg.angular.z = 0.0f;
            ros.Publish("cmd_vel", msg);
        }
    }
}
```

---

# 🎥 Example: Displaying Depth Image in Unity

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class DepthListener : MonoBehaviour
{
    public Texture2D depthTexture;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<ImageMsg>("/camera/depth", DepthCallback);
    }

    void DepthCallback(ImageMsg msg)
    {
        depthTexture.LoadRawTextureData(msg.data);
        depthTexture.Apply();
    }
}
```

---

# 🎯 Summary

✔ Unity gives next-gen visualization
✔ ROS-TCP creates two-way communication
✔ Scripts let Unity control real robots
✔ You can build amazing dashboards and cinematic demos
