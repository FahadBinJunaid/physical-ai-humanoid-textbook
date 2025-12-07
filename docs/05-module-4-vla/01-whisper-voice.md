# 5.1 Voice-to-Action (Whisper) 🗣️🤖

Voice-to-Action allows robots to **interpret spoken commands** and convert them into actions.
Using OpenAI Whisper, robots can **listen, transcribe, and execute** commands autonomously.

---

# 🌟 Why Voice-to-Action?

* Enables **hands-free control** 🎮
* Makes robots interactive and user-friendly 🤝
* Integrates with LLM planning for **complex task execution** 🧠
* Useful for home, industrial, or research robots 🏡🏭

---

# 🔍 1. Speech Recognition Workflow

1. Capture audio using microphone 🎤
2. Convert audio to text via Whisper transcription 📝
3. Process text commands and map to robot actions 🤖

---

# 🛠️ Example: Capturing Audio and Transcribing

```python
import whisper

# Load Whisper model
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("command.wav")
print(f"Transcribed text: {result['text']}")
```

**Explanation**:

* `base` model balances accuracy and speed
* Output text can now be parsed to determine robot actions

---

# 🔍 2. Mapping Commands to Actions

* Parse text commands: "Move forward", "Pick object", "Turn left"
* Map each command to a **ROS 2 topic or function**

```python
command_text = result['text'].lower()

if "forward" in command_text:
    # Publish to /cmd_vel
    ros_publish("/cmd_vel", linear_x=0.5)
elif "stop" in command_text:
    ros_publish("/cmd_vel", linear_x=0.0)
elif "pick" in command_text:
    control_arm("pick")
```

---

# 🧠 3. Integration with ROS 2

* Transcribed text → ROS 2 topics
* Use `geometry_msgs/Twist` for movement
* Use custom messages for manipulators or sensors

```python
from geometry_msgs.msg import Twist
import rclpy

# Create ROS node
node = rclpy.create_node("voice_cmd_node")
pub = node.create_publisher(Twist, "/cmd_vel", 10)

# Example: publish forward motion
twist = Twist()
twist.linear.x = 0.5
pub.publish(twist)
```

---

# 🎯 Summary

✔ Audio captured and transcribed with Whisper 🗣️
✔ Text commands mapped to robot actions 🤖
✔ ROS 2 enables real-time control and integration ⚡
✔ Voice-to-Action creates natural, interactive robot experiences 🎮🤝
