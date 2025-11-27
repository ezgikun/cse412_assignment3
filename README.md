# 🤖 CSE412 Robotics - Assignment 3: ROS 2 Humble Mini Project in Docker

## 📝 Project Overview

This repository contains the solution for the BYM412 Robotics Course's third assignment. The objective of this project was to develop a **three-node ROS 2 Humble application** written in Python, containerize the entire system using **Docker**, and demonstrate full functionality—including continuous data publishing and custom service handling—from within the isolated Docker environment.
The system architecture features a **Publisher-Subscriber** data pipeline complemented by a **Service Server** for interactive control and status checks.

## ✨ Key Features & Requirements

The application successfully implements the following core functionalities as required by the assignment:

  * **Three ROS 2 Nodes (Python):** `sensor_publisher`, `data_processor`, and `command_server`.
  * **Custom Service Interface:** Implementation of a custom service (`ComputeCommand.srv`) defined to accept a `float64` input and return a `string` output.
  * **Service Logic:** The `command_server` uses threshold logic: **`input > 10` → "HIGH"** and **`input <= 10` → "LOW"**.
  * **Single Launch File:** All three nodes are launched simultaneously using `my_project.launch.py`.
  * **Full Containerization:** The entire application is built, configured, and executed using a single `Dockerfile` and `entrypoint.sh` on the `ros:humble-ros-base` image.

## 🧠 Node Descriptions and Data Flow

| Package Name | Executable | Type | Responsibility / Data Flow |
| :--- | :--- | :--- | :--- |
| `sensor_publisher_pkg` | `sensor_publisher` | Publisher | [cite\_start]Publishes synthetic sensor data (random `Float32` values) to the **`/sensor_value`** topic every 0.1 seconds (10 Hz) |
| `data_processor_pkg` | `data_processor` | Subscriber/Publisher | [cite\_start]Subscribes to **`/sensor_value`**, processes the data (multiplies by 2), and publishes the result to **`/processed_value`**. |
| `command_server_pkg` | `command_server` | Service Server | [cite\_start]Implements the **`/compute_command`** service, listening for requests using the custom `ComputeCommand` service type. |

## 📁 Project Structure

The project adheres to the specified directory structure, placing all source code and configuration files logically within a single workspace:

```
cse412_a3/
├── Dockerfile                  # Instructions for building the environment and the ROS workspace.
├── entrypoint.sh               # Startup script executed by Docker ENTRYPOINT to launch the ROS system.
├── launch/                     # Contains the my_project.launch.py file.
├── src/                        # ROS 2 workspace source directory.
│   ├── custom_interfaces/      # Defines ComputeCommand.srv (built using ament_cmake).
│   ├── sensor_publisher_pkg/   # Source for the publisher node.
│   ├── data_processor_pkg/     # Source for the processor node.
│   └── command_server_pkg/     # Source for the service server node.
├── README.md                   # This file.
└── SSF_HASH.txt                # Student Signature File output for verification.
```

## 🛠️ Setup and Installation (Using Docker)

The host machine only requires **Docker** to be installed. [cite\_start]ROS 2 installation on the host is **not required**.

### Prerequisites

  * Docker (latest version)
  * Git

### Step 1: Clone the Repository

Clone this repository to your local machine:

```bash
git clone https://github.com/ezgikun/cse412_assignment3.git
cd cse412_assignment3
```

### Step 2: Build the Docker Image

The `Dockerfile` handles installing all necessary build tools (`ament_cmake`, `colcon`), resolving dependencies (`rosdep`), and building the custom interfaces and Python nodes.

```bash
# Build the image and tag it as 'myrosapp'
docker build -t myrosapp .
```

### Step 3: Run the Container

[cite\_start]The container is configured to automatically source the ROS environment and execute the launch file (`my_project.launch.py`) via the `entrypoint.sh` script when it starts.

```bash
# Run the container (use --rm to clean up when stopped)
docker run --rm myrosapp
```

*In the terminal running this command, you should immediately start seeing the output from the `sensor_publisher` and `data_processor` nodes.*

-----

## ✅ Usage and Verification

All verification steps must be performed in a separate terminal **inside the running Docker container**.

### Step 1: Access the Container Shell

Open a new terminal, identify the running container ID, and open an interactive bash shell inside it:

```bash
# Find the Container ID (e.g., dcfe45529b90)
docker ps

# Enter the container shell
docker exec -it <CONTAINER_ID> bash
```

### Step 2: Source the ROS Environment

Inside the container, manually source the environment to make `ros2` commands available:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Step 3: Run Verification Commands

Execute the following commands to confirm node functionality and data flow:

#### 1\. Verify Active Nodes and Topics

Confirms all three nodes are running and that the required topics are available.

```bash
ros2 topic list
```

#### 2\. Verify Data Flow and Processing
i
Confirms that the `data_processor` is subscribing to the raw data and publishing the processed (multiplied by 2) result.

```bash
ros2 topic echo /processed_value
```

*(You should see a continuous stream of numerical values.)*

#### 3\. Verify Custom Service Operation

Confirms the `command_server` node is running and correctly responding to requests based on the threshold logic.

```bash
# Input 12.5 (> 10) should return "HIGH"
ros2 service call /compute_command custom_interfaces/ComputeCommand "input: 12.5"
```

*(**Expected Output:** `output='HIGH'`)*

-----

## 🔗 Deliverables

This project repository and associated report fulfill the following deliverables:

  * **Source Code, Dockerfile, and Launch File** 
  * **README** and **`SSF_HASH.txt`**
  * **PDF Report** (`A3_<StudentID>_<NameSurname>.pdf`)
  * **YouTube Video Link** (Unlisted)
