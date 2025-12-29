# HomeCleanerBot: Autonomous Indoor Cleaning Robot Simulation

This project involves the development of an autonomous indoor cleaning robot simulation called **HomeCleanerBot**. It integrates SLAM for environment mapping, Nav2 for autonomous navigation, a coverage planner for systematic cleaning, and a custom RViz interface for robot control—all encapsulated within a standalone Docker environment.

## 👥 Authors
- **Elif Ceyda Civelek** - [elif.civelek@istun.edu.tr]
- **Mehmet Sefa Toksoy** - [mehmet.toksoy@istun.edu.tr]

---

## 🛠 Tech Stack
- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble
- **Simulator:** Gazebo Harmonic
- **Mapping:** SLAM Toolbox (Async Mode)
- **Navigation:** Nav2 (Navigation 2 Stack)
- **Containerization:** Docker

---

## 🚀 Getting Started

### Prerequisites
- Docker installed on your host machine.
- X11 server access (for GUI forwarding). Run the following command on your host terminal to allow Docker to open Gazebo/RViz windows:
  ```bash
  xhost +local:root
  ```

### Installation & Build
1. Clone the repository:
   ```bash
   git clone https://github.com/elifcivelek12/HomeCleaningBot.git
   cd HomeCleaningBot
   ```
2. Build the Docker image:
   ```bash
   docker build -t homecleanerbot .
   ```

### Running the Simulation
Execute the project with a single command:
```bash
docker run --rm -it \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    homecleanerbot
```

---

## 🧩 Key Components

### 1. Home Layout (SDF)
A custom-designed **2+1 apartment** created using SDF. It includes:
- Living Room (Salon), Kitchen area, and two Bedrooms.
- Realistic wall structures with headers above doors.
- Essential furniture and a dedicated **Docking Station**.

### 2. Robot Design (Xacro)
The `cleaning_bot` is a differential drive robot featuring:
- **LiDAR Sensor:** Configured for 360° environment scanning.
- **Stabilizers:** Front and back caster wheels for high-speed stability.
- **Physics Plugins:** Uses Gazebo Harmonic's `DiffDrive` and `Sensors` systems.

### 3. SLAM & Navigation
- **Mapping:** Utilizes `slam_toolbox` to generate high-resolution occupancy grids.
- **Nav2:** Fully configured navigation stack with AMCL localization.
- **Coverage Planner:** A Boustrophedon-based path planner that ensures the robot visits every reachable free space in the home.

### 4. RViz Command Interface
A custom Python-based **GUI Control Panel** is integrated into the workspace. Users can trigger the following commands directly:
- **START CLEANING:** Initiates the coverage planning and cleaning cycle.
- **STOP CLEANING:** Cancels the current navigation task immediately.
- **RETURN TO DOCK:** Commands the robot to autonomously navigate back to its charging station.

---

## 📂 Project Structure
```text
.
├── src
│   ├── home_cleaning_robot   # Main robot logic, URDF, Nav2 configs, and GUI
│   └── home_cleaner_sim     # Gazebo world and environment files
├── Dockerfile                # Standalone project image recipe
├── docker-compose.yaml       # Development environment configuration
└── README.md
```

---
**Istanbul Health and Technology University**
*Department of Computer/Software Engineering - CSE412 Robotics Course*
