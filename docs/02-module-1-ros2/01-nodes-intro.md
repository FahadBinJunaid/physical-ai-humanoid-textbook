# 2.1 Nodes, Topics, and Messages 🤖🔌

In ROS 2, **everything begins with Nodes**. A robot is basically a collection of many small programs (nodes) that communicate with each other through **topics**, **messages**, **services**, or **actions**.

This chapter explains the foundation of ROS 2 communication.

---

# 🧠 What is a Node?

A **Node** in ROS2 is a single running program — like a LEGO block in your robot’s brain.

Examples of nodes:

- A *camera node* 📷  
- A *motor controller node* ⚙️  
- An *AI detector node* 🧠  
- A *navigation node* 🧭  

Each node does one job, but together they form an entire robot system.

---

# 🛰️ Topics: Streaming Data Between Nodes

A **Topic** is like a live broadcast channel.

Nodes communicate by:

- **Publishing** on a topic (sending messages)  
- **Subscribing** to a topic (receiving messages)  

Examples:

Topic Name | Purpose  
--- | ---  
`/camera/image_raw` | Camera frames  
`/scan` | Lidar readings  
`/cmd_vel` | Movement commands  

Communication is *asynchronous*, meaning publishers and subscribers do not wait for each other.

---

# 💬 Messages: The Data You Send

Messages define **what kind of data** is being sent over a topic.

Examples:

- `std_msgs/String` → Text messages  
- `sensor_msgs/Image` → Camera images  
- `geometry_msgs/Twist` → Robot velocity  

A topic can only use **one specific message type**.

---

# 🔁 How Data Flows in ROS2

Example system:

Camera Node ───► Image Topic ───► Object Detection Node ───► Decision Topic ───► Motor Node

python
Copy code

Every part is a small, modular block — flexible and scalable.

---

# 📘 Code Example (Publisher + Subscriber)

```python
# 📨 Simple ROS2 Publisher + Subscriber Example

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Publisher Node
class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "chat", 10)
        self.timer = self.create_timer(1.0, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = "👋 Hello from the publisher node!"
        self.pub.publish(msg)

# Subscriber Node
class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.sub = self.create_subscription(String, "chat", self.callback, 10)

    def callback(self, msg):
        print("📩 Received:", msg.data)

rclpy.init()
rclpy.spin(Talker())
rclpy.spin(Listener())