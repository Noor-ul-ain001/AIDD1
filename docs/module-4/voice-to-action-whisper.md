---
sidebar_position: 1
---

# Voice-to-Action: Using OpenAI Whisper for Voice Commands

The ability to control a robot with your voice is a classic trope of science fiction, but it's now becoming a reality. Thanks to advances in automatic speech recognition (ASR), we can now build systems that can understand spoken language and translate it into robotic actions. OpenAI's Whisper is a state-of-the-art ASR model that is particularly well-suited for this task.

<details>
  <summary>Quick Summary</summary>
  <div>
    <p>
      This section explores how to use OpenAI's Whisper model to create a voice-controlled robot.
    </p>
    <ul>
      <li><strong>What is Whisper?</strong> A powerful and accurate automatic speech recognition (ASR) model from OpenAI.</li>
      <li><strong>The Voice-to-Action Pipeline:</strong> Understand the steps involved in converting a spoken command into a robotic action.</li>
      <li><strong>Integrating Whisper with ROS 2:</strong> Learn how to create a ROS 2 node that uses Whisper to transcribe audio and publish the resulting text.</li>
      <li><strong>From Text to Action:</strong> Discover how to process the transcribed text and convert it into a command that the robot can understand.</li>
    </ul>
  </div>
</details>

## Why Whisper?

Whisper is an excellent choice for robotics applications for several reasons:

-   **High Accuracy:** Whisper has been trained on a massive dataset of diverse audio, making it very accurate in a wide range of conditions.
-   **Robustness to Noise:** It is designed to be robust to background noise, which is a common problem in real-world robotics environments.
-   **Multilingual Support:** Whisper can transcribe audio in many different languages.
-   **Open Source:** The model and its code are open source, allowing you to run it locally on your own hardware.

## The Voice-to-Action Pipeline

The process of converting a spoken command into a robotic action can be broken down into four main steps:

1.  **Audio Capture:** Capture audio from a microphone attached to the robot or in the environment.
2.  **Speech-to-Text:** Use Whisper to transcribe the captured audio into a text string.
3.  **Natural Language Understanding (NLU):** Process the text to extract the user's intent and any relevant entities. For example, in the command "robot, please pick up the red ball," the intent is "pick up," and the entities are "red ball."
4.  **Action Execution:** Translate the extracted intent and entities into a specific ROS 2 action or service call that the robot can execute.

## Integrating Whisper with ROS 2

To integrate Whisper into your ROS 2 system, you can create a node that:

1.  **Subscribes to an audio topic:** ROS 2 has standard message types for audio data (`audio_common_msgs/AudioData`).
2.  **Calls the Whisper API:** When it receives audio data, the node sends it to the Whisper model for transcription. You can run the Whisper model locally or use OpenAI's hosted API.
3.  **Publishes the transcribed text:** The node then publishes the resulting text string to a new topic (e.g., `transcribed_text`).

Here's a conceptual example of what this node might look like:

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import whisper

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.subscription = self.create_subscription(
            AudioData,
            'audio',
            self.audio_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'transcribed_text', 10)
        self.model = whisper.load_model("base") # Or other model size

    def audio_callback(self, msg):
        # This is a simplified example. You would need to handle the audio
        # data format and convert it to a format that Whisper can process.
        audio_data = msg.data

        # Transcribe the audio
        result = self.model.transcribe(audio_data)
        transcribed_text = result["text"]

        # Publish the text
        text_msg = String()
        text_msg.data = transcribed_text
        self.publisher_.publish(text_msg)
        self.get_logger().info('Whisper heard: "%s"' % transcribed_text)

# ... main function to run the node ...
```

## From Text to Action

Once you have the transcribed text, you need to process it to understand the user's command. This can be as simple as looking for keywords (e.g., "forward," "stop," "pick up") or as complex as using a large language model (LLM) to perform more advanced natural language understanding.

For example, a simple keyword-based approach might look like this:

-   If the text contains "go forward," publish a command to the robot's navigation system.
-   If the text contains "stop," publish a command to stop the robot.
-   If the text contains "pick up," trigger a manipulation task.

In the next section, we'll explore how to use LLMs to create more sophisticated and flexible cognitive planning systems.
