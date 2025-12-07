# 2.4 Services and Actions 🔁⚡

Nodes in ROS2 communicate in different ways.  
So far, we learned **topics** (continuous streams).  
Now we explore **Services** and **Actions**, which allow more controlled communication.

---

# 🔧 Services — Synchronous Requests

A service is like calling a function on another node:

- You send a request  
- You wait  
- You receive a response  

Used for tasks that are QUICK:

Examples:
- Resetting sensors  
- Querying battery  
- Getting robot state  
- Triggering a one-time command  

Services **block** until they get an answer.

---

# 🏃 Actions — Asynchronous, Long-Running Tasks

Actions are used for tasks that take time:

Examples:
- Navigate to goal  
- Move robotic arm  
- Follow a path  
- Execute complex behaviors  

Features of Actions:

- Send goal  
- Get feedback  
- Cancel goal  
- Get result  

Actions run **asynchronously**.

---

# 🔄 Difference Table

Feature | Topics | Services | Actions
--- | --- | --- | ---
Communication | Continuous stream | Request/Response | Long task w/ feedback
Blocking? | No | Yes | No
Use Case | Sensors, control loops | Quick commands | Movement, navigation

---

# 📘 Service Example (Add Two Numbers)

```python
# ➕ ROS2 Service Example: Add Two Numbers

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__("add_server")
        self.srv = self.create_service(AddTwoInts, "add_numbers", self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        print(f"🧮 {request.a} + {request.b} = {response.sum}")
        return response

def main():
    rclpy.init()
    node = AddServer()
    rclpy.spin(node)

main()
