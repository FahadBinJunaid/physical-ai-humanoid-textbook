# 2.2 Building Agents with rclpy 🐍🤖

In ROS2, **agents** are intelligent units built from nodes that perform tasks, respond to events, and generate structured behaviors.

This chapter teaches you how to create those agents using `rclpy` — the official ROS2 Python client library.

---

# 🎯 What is an "Agent"?

A robot "agent" is:

- A node + logic  
- A node + perception  
- A node + decision-making  

An agent is more than a simple node — it contains **behavior**.

Example agents:

- 🤖 Patrol Agent  
- 🛑 Obstacle Avoidance Agent  
- 🧭 Navigation Agent  
- 🧠 Object Detection Agent  

---

# 🧩 Key rclpy Concepts

### 🔹 Node  
The main execution unit.

### 🔹 Timer  
Runs a function every X seconds.

### 🔹 Subscription  
Listens to topic data.

### 🔹 Publisher  
Sends messages to topics.

### 🔹 Parameters  
Configurable values (speed, thresholds, etc.)

---

# 🏗️ Building a Basic Python Agent

Steps:

1️⃣ Import ROS2 libraries  
2️⃣ Create a Node class  
3️⃣ Add publishers/subscribers  
4️⃣ Add logic  
5️⃣ Spin the node  

---

# 📘 Full Agent Example (Obstacle Monitor)

```python
# 🤖 Obstacle Monitoring Agent (Complete Example)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ObstacleAgent(Node):
    def __init__(self):
        super().__init__("obstacle_agent")

        self.subscription = self.create_subscription(
            Float32,
            "distance_sensor",
            self.callback,
            10,
        )

    def callback(self, msg):
        distance = msg.data
        print(f"📡 Distance: {distance}m")

        if distance < 0.5:
            print("🚫 Obstacle detected! WARNING!")
        else:
            print("🟢 Path clear.")

def main():
    rclpy.init()
    node = ObstacleAgent()
    rclpy.spin(node)

main()
