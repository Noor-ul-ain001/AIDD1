---
sidebar_position: 2
---

# Cognitive Planning with LLMs

The previous section showed how to convert speech to text. But how do we go from a simple text command to a complex sequence of actions? This is where Large Language Models (LLMs) come in. LLMs like GPT-3 and PaLM can be used to perform "cognitive planning" - the process of translating a high-level, natural language goal into a concrete plan of action for a robot.

<details>
  <summary>Quick Summary</summary>
  <div>
    <p>
      This section explores the exciting field of using Large Language Models (LLMs) for robotic planning.
    </p>
    <ul>
      <li><strong>What is Cognitive Planning?</strong> The process of translating a high-level, natural language command into a sequence of robotic actions.</li>
      <li><strong>Why use LLMs?</strong> LLMs have a deep understanding of language and common sense, which allows them to create plausible plans for a wide variety of tasks.</li>
      <li><strong>The LLM Planning Pipeline:</strong> Learn the steps involved in using an LLM for robotic planning, from crafting the prompt to executing the plan.</li>
      <li><strong>Prompt Engineering:</strong> Discover the importance of "prompt engineering" and how to design a prompt that gives the LLM the context it needs to generate a good plan.</li>
    </ul>
  </div>
</details>

## The Power of Common Sense

One of the key advantages of using LLMs for robotic planning is their vast amount of "common sense" knowledge. An LLM knows that to "clean the room," you might need to pick up toys, put away books, and wipe down surfaces. This allows you to give the robot very high-level commands, and the LLM can figure out the necessary steps to achieve the goal.

This is a major departure from traditional robotic planning, which typically requires a human to manually specify every step of the plan.

## The LLM Planning Pipeline

The process of using an LLM for cognitive planning typically involves the following steps:

1.  **Prompt Engineering:** This is the most critical step. You need to create a "prompt" that gives the LLM all the information it needs to generate a good plan. This includes:
    -   The high-level goal (e.g., "Clean the room").
    -   A description of the robot's capabilities (e.g., "You can navigate to a location, pick up an object, and place it in a container").
    -   Information about the current state of the environment (e.g., "There is a red ball on the floor and a blue cube on the table").

2.  **LLM Inference:** You send the prompt to the LLM and get back a response. The response will typically be a sequence of steps that the robot should take.

3.  **Plan Parsing and Validation:** You need to parse the LLM's response and validate that it is a feasible plan. This might involve checking that the robot is capable of performing each action and that the objects and locations mentioned in the plan actually exist.

4.  **Plan Execution:** You translate each step of the plan into a specific ROS 2 action or service call and execute it on the robot.

## Prompt Engineering for Robotic Planning

The quality of the plan you get from an LLM is highly dependent on the quality of your prompt. Here are some tips for effective prompt engineering:

-   **Be Specific:** Provide as much detail as possible about the robot's capabilities and the environment.
-   **Use Examples:** You can use "few-shot" prompting, where you provide a few examples of high-level goals and their corresponding plans in the prompt. This can help the LLM to understand the desired output format and style.
-   **Constrain the Output:** You can ask the LLM to generate the plan in a specific format, such as a numbered list or a JSON object. This can make it easier to parse the plan.

Here's an example of a simple prompt for a robot that can navigate and pick up objects:

```
You are a helpful assistant that controls a robot. The robot has the following capabilities:
- navigate_to(location)
- pick_up(object)
- place(object, location)

The current state of the environment is:
- A red ball is on the floor.
- A blue cube is on the table.
- The robot is at the charging station.
- There is a box next to the table.

Your goal is to "Clean the room".

Please provide a plan to achieve this goal.
```

The LLM might respond with a plan like this:

1.  `navigate_to(floor)`
2.  `pick_up(red_ball)`
3.  `navigate_to(box)`
4.  `place(red_ball, box)`
5.  `navigate_to(table)`
6.  `pick_up(blue_cube)`
7.  `navigate_to(box)`
8.  `place(blue_cube, box)`

By combining the power of LLMs for high-level reasoning with the precision and reliability of ROS 2 for low-level control, you can create truly intelligent and capable robots.