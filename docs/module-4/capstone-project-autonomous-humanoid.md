---
sidebar_position: 3
---

# Capstone Project: The Autonomous Humanoid

This capstone project brings together everything you've learned in previous modules to create an autonomous humanoid robot that can understand voice commands, plan a sequence of actions, and execute them in a simulated environment.

<details>
  <summary>Quick Summary</summary>

  <p>
    This section outlines the final capstone project, where you will build an end-to-end system for an autonomous humanoid robot.
  </p>

  <ul>
    <li><strong>The Goal:</strong> To create a simulated humanoid robot that can respond to a high-level voice command like "clean the room."</li>

    <li><strong>The Pipeline:</strong> The project will integrate all the key technologies from the course:</li>

    <ul>
      <li><strong>Voice Command:</strong> Use Whisper for speech-to-text.</li>
      <li><strong>Cognitive Planning:</strong> Use an LLM to generate a plan.</li>
      <li><strong>Navigation:</strong> Use Nav2 to plan a path for the robot's body.</li>
      <li><strong>Perception:</strong> Use computer vision to identify and locate objects.</li>
      <li><strong>Manipulation:</strong> Use a whole-body controller to pick up and place objects.</li>
    </ul>

    <li><strong>The Simulation Environment:</strong> Implement the project in a high-fidelity environment like NVIDIA Isaac Sim.</li>
  </ul>

</details>

## The Challenge

The goal of the capstone project is to create a system that can successfully execute the following scenario:

1. The user gives a voice command to the robot, such as "Clean the room."
2. The robot's microphone captures the audio.
3. The audio is transcribed into text using OpenAI's Whisper.
4. The transcribed text is sent to a Large Language Model (LLM) to generate a high-level plan.
5. The plan is parsed and translated into a sequence of ROS 2 actions.
6. The robot navigates to the first object's location using Nav2.
7. The robot uses its camera and a computer vision model to identify and get the precise location of the object.
8. The robot uses its whole-body controller to pick up the object.
9. The robot navigates to a designated "storage" location (e.g., a box).
10. The robot places the object in the storage location.
11. The robot repeats this process until all objects have been put away.

## The System Architecture

The system will be composed of several interconnected ROS 2 nodes:

- **Audio Input Node:** Captures audio from the microphone and publishes it to a topic.
- **Whisper Node:** Subscribes to the audio topic, transcribes the audio, and publishes the text.
- **LLM Planner Node:** Subscribes to the text topic, calls the LLM to generate a plan, and publishes the plan as a sequence of actions.
- **Plan Executor Node:** Subscribes to the plan topic and orchestrates execution by calling ROS 2 actions and services.
- **Navigation Node:** An instance of Nav2, configured for the humanoid robot.
- **Perception Node:** A computer vision node that subscribes to the robot's camera feed to detect and locate objects.
- **Manipulation Node:** A whole-body controller that can execute picking and placing actions.

## The Simulation Environment

This project will be implemented in a high-fidelity simulation environment like NVIDIA Isaac Sim, enabling you to:

- **Create a realistic environment:** Build a room with furniture, objects, and realistic lighting.
- **Use a high-quality robot model:** Utilize a detailed humanoid robot model with accurate kinematics and dynamics.
- **Generate synthetic sensor data:** Produce photorealistic camera images and sensor outputs to test perception algorithms.

## Putting It All Together

This capstone project is a challenging but rewarding undertaking that requires integrating all skills learned throughout the book. By the end, you will have built a complete, end-to-end system for an autonomous humanoid robot — from high-level voice commands to low-level motor control.
