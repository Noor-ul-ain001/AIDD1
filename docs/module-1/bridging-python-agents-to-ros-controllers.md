---
sidebar_position: 2
---

# Bridging Python Agents to ROS Controllers with rclpy

Now that you understand the basics of ROS 2, let's explore how to connect the world of AI and machine learning with robotics. This is where `rclpy`, the ROS 2 client library for Python, comes in. `rclpy` allows you to write ROS 2 nodes in Python, enabling seamless integration with popular AI/ML frameworks like TensorFlow, PyTorch, and scikit-learn.

<details>
  <summary>Quick Summary</summary>
  <div>
    <p>
      This section focuses on using `rclpy` to connect Python-based AI agents to ROS 2 controllers.
    </p>
    <ul>
      <li><strong>`rclpy` Basics:</strong> Learn how to initialize `rclpy`, create nodes, and handle shutdown.</li>
      <li><strong>Publishing and Subscribing:</strong> Use `rclpy` to publish data (e.g., commands from an AI agent) and subscribe to data (e.g., sensor readings for the agent).</li>
      <li><strong>A Practical Example:</strong> See a conceptual example of a simple AI agent that subscribes to sensor data, processes it, and publishes control commands.</li>
    </ul>
  </div>
</details>

## Why Bridge AI to ROS?

The power of combining AI with ROS is immense. You can create intelligent robots that can:

-   Learn from their environment.
-   Make decisions based on complex sensor data.
-   Interact with humans in a more natural way.
-   Perform tasks that are too difficult or dangerous for humans.

`rclpy` is the bridge that makes this all possible, allowing your Python-based AI agents to communicate with the rest of your ROS 2 system.

## Key `rclpy` Concepts

To get started with `rclpy`, you'll need to understand a few key concepts:

-   **Initialization and Shutdown:** Every `rclpy` program starts by initializing the `rclpy` library and ends by shutting it down.
-   **Creating Nodes:** As you've seen, you can create a node by inheriting from `rclpy.node.Node`.
-   **Publishers and Subscribers:** You can create publishers and subscribers to send and receive messages on ROS 2 topics.
-   **Timers:** Timers allow you to execute a callback function at a regular interval.
-   **Executors:** An executor is responsible for running the callbacks in your nodes (e.g., subscription callbacks, timer callbacks). The most common executor is `rclpy.spin()`.

## Conceptual Example: An AI-Powered Robot

Let's imagine you want to create a robot that avoids obstacles using a simple AI agent. Here's how you might structure the code:

1.  **Sensor Node:** A node that publishes data from a distance sensor (e.g., an ultrasonic sensor or a LiDAR).
2.  **AI Agent Node:** A Python node that subscribes to the sensor data. This node would contain your AI/ML model. Based on the sensor readings, the model would decide whether to turn left, right, or move forward. The node would then publish these commands to a "command" topic.
3.  **Controller Node:** A node that subscribes to the "command" topic and translates the high-level commands ("turn left") into low-level motor commands.

Here's a simplified version of what the AI Agent node might look like:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Assuming sensor data is a float
from std_msgs.msg import String   # Assuming commands are strings

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.sensor_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)

    def sensor_callback(self, msg):
        # In a real-world scenario, you would feed this data into your AI model
        sensor_distance = msg.data

        command = String()
        if sensor_distance < 0.5:  # If obstacle is too close
            command.data = 'turn_left'
        else:
            command.data = 'move_forward'

        self.publisher_.publish(command)
        self.get_logger().info('AI Agent says: "%s"' % command.data)

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()
    rclpy.spin(ai_agent_node)
    ai_agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates the basic principle of bridging a Python agent with a ROS 2 system. The AI logic is contained within the `sensor_callback`, and it communicates its decisions to the rest of the robot through a ROS 2 topic.
