---
sidebar_position: 1
---

# ROS 2 Nodes, Topics, and Services

Welcome to the core of the Robotic Nervous System: ROS 2. In this section, we'll explore the fundamental building blocks of any ROS 2 application: nodes, topics, and services.

<details>
  <summary>Quick Summary</summary>
  <div>
    <p>
      This section introduces the foundational concepts of ROS 2.
    </p>
    <ul>
      <li><strong>Nodes:</strong> The main computational units in ROS 2. Each node should be responsible for a single, module purpose (e.g., controlling a wheel, reading a sensor).</li>
      <li><strong>Topics:</strong> The primary method for nodes to exchange data. Nodes can publish messages to a topic or subscribe to a topic to receive messages. This is an asynchronous, one-to-many communication model.</li>
      <li><strong>Services:</strong> A request/response communication model. One node (the client) sends a request to another node (the server) and waits for a response. This is a synchronous, one-to-one communication model.</li>
    </ul>
  </div>
</details>

## ROS 2 Nodes

A node is the smallest unit of computation in a ROS 2 system. Think of it as a small, specialized program that performs a single task. For example, you might have a node for each of the following:

-   Controlling a robot's wheel
-   Reading data from a laser scanner
-   Planning a path for the robot to follow
-   Broadcasting the robot's current position

Nodes are typically written in Python or C++ and are organized into packages.

### Example: A Simple Publisher Node

Here's an example of a simple ROS 2 node written in Python that publishes a "Hello, World!" message to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher = HelloWorldPublisher()
    rclpy.spin(hello_world_publisher)
    hello_world_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Topics

Topics are named buses over which nodes exchange messages. Topics are one of the main ways that data is moved between nodes and other parts of the system.

-   **Publishers:** A node can publish a message to a topic.
-   **Subscribers:** A node can subscribe to a topic to receive messages.

Multiple nodes can publish to and subscribe to the same topic. This creates a decoupled architecture where nodes don't need to know about each other directly.

### Message Types

Every topic has a specific message type. ROS 2 comes with a rich set of standard message types (`std_msgs`, `sensor_msgs`, etc.), and you can also define your own custom message types.

## ROS 2 Services

Services are another way for nodes to communicate with each other. Unlike topics, which use a publish/subscribe model, services use a request/response model.

-   **Service Server:** A node that provides a service.
-   **Service Client:** A node that requests a service.

When a client calls a service, it sends a request message and waits for a response message from the server. This is a synchronous communication mechanism, making it ideal for remote procedure calls (RPCs).

### Example: A Simple Service Client

Here's an example of a simple ROS 2 service client.

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(SetBool, 'set_boolean')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, value):
        self.req.data = value
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_client = SimpleServiceClient()
    response = service_client.send_request(True)
    service_client.get_logger().info(
        'Result of SetBool: for "%r" got "%r"' % (True, response.success))
    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```