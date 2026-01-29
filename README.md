# Advantech AMR Perceptor

The Advantech AMR Perceptor is an advanced AI intelligence layer designed as an extension of the Advantech AMR DevKit. While the DevKit provides the "Body" (hardware drivers and physical models), the Perceptor provides the "Brain".

This package is a demo scenario that adds high-level perception and reasoning to the robot. It depends entirely on the hardware foundation established by the AMR DevKit. By using the Perceptor, your robot can identify objects in 3D space, understand its surroundings, and communicate with users through a chat interface.

## Key Features

- **Intelligence Extension**: This project is built specifically to extend the AMR DevKit. It transforms raw sensor data into actionable AI insights.

- **Built-in Data Optimization**: Implements specialized nodes for image resizing and pointcloud downsampling. This ensures that the AI models run efficiently on the Advantech MIC-732-AO without overloading the system.

- **3D Object Detection**: Uses YOLOv8 to detect objects (like people, vehicles, or tools) and calculates their exact 3D position relative to the robot.

- **Natural Language Reasoning**: Integrates Llama.cpp and an MCP (Model Context Protocol) Server. This allows the robot to "reason" about what it sees and answer questions like "Where is the person?" in plain English.

- **Interactive Web UI**: Provides a user-friendly chat interface via Open-WebUI, allowing you to interact with the AMR from any web browser to enhance demo experience.

## System Architecture

The AMR Perceptor acts as the "Intelligence Layer" sitting on top of the AMR DevKit "Hardware Layer." The following diagram illustrates the data flow from physical sensors to the user interface:

- **Input Layer (AMR DevKit Dependency)**: Receives raw camera (GMSL/Fisheye) and LiDAR data from the AMR DevKit drivers.

- **Vision Engine**: The YOLOv8 node analyzes the optimized data to detect objects and determine their 3D coordinates using the spatial transforms provided by the DevKit.

- **Semantic Context (MCP)**: The ROS Agent collects these 3D detections and serves them to the Large Language Model (LLM) through the Model Context Protocol (MCP).

- **User Interaction**: The LLM processes user queries via Open-WebUI, providing natural language answers based on the robot's real-time environmental data.

## Installation Guide

The Perceptor package uses Docker to manage complex AI dependencies and ensure a stable environment.

### 1. Install AMR DevKit

The Perceptor requires the AMR DevKit to be installed and configured first. Please refer to the for hardware setup and driver installation.

### 2. Install Docker Engine

If Docker is not installed, please follow the [instructions](https://github.com/advantech-EdgeAI/AMR_DevKit/issues/1) to set it up.

## Usage / Quick Start

### 1. Initialize the Hardware (DevKit)

In a new terminal, launch the AMR DevKit drivers. Using `yolo_rviz:=true` opens a viewer that shows AI detection results.:

```bash
ros2 launch amr_description bringup.launch.py yolo_rviz:=true preprocess:=true
```

⚠️ **Important Notice**

To prevent system crashes or errors, always close the RViz window first before using `Ctrl+C` to stop the processes in your terminals.

### 2. Start YOLO, MCP, llama.cpp, and Web UI

```bash
$ docker compose up -d

[+] Running 4/4
 ✔ Container llamacpp_server_container  Healthy    11.4s
 ✔ Container yolo_container             Started     1.0s
 ✔ Container mcp_container              Healthy     5.9s
 ✔ Container open_webui_container       Started    12.1s
```

### 3. Open Web UI in browser

Navigate to the URL: `http://<IP of your device>:9988`

### 4. Open Admin Panel

### 5. Configure llama.cpp

Specifiy `http://llamacpp:10000/v1` as URL. Use the 🔄 button (next to the URL) to test the connection with llama.cpp.

### 6. Configure MCP

Specify `http://mcpo:9000` as URL and "ROS mcp server" as name. Use the 🔄 button (next to the URL) to test the connection with MCP server.

### 7. Import Default Prompt Suggestions

Import the predifined prompts by uploading the file: `ros_agent_prompt_suggestions.json`.

### 8. Import Workspace

Import the predifined workspace by uploading the file: `ros_agent_model.json`. The ROS Agent model will be added.

### 9. Edit Model Setting

Select base model for ROS Agent: `gpt-oss-20b-Q4_K_M.gguf`

### 10. Interact with AMR

Use the suggested prompt: Get the surroundings. Ensure all the services are working together by checking objects' 3D positions.

### 12. Close All Services

```bash
docker compose down
```

