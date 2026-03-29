# Drone State Navigation & Autonomous Landing Sim

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%2FIgnition-F58113)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)

A **fully autonomous drone flight stack** built from scratch using **ROS 2 (Python)** and **Gazebo Harmonic/Ignition**. This simulation features a custom physics world, a complete computer vision pipeline with a live Heads-Up Display (HUD), finite state machine (FSM) logic, visual servoing for precision landing, and a dynamic Command & Control (C2) teleporter — all orchestrated across four isolated, composable ROS 2 nodes.

---

## Table of Contents

- [System Architecture](#️-system-architecture)
- [Prerequisites](#️-prerequisites)
- [Installation & Build](#️-installation--build)
- [Usage / How to Run](#-usage--how-to-run)

---

## System Architecture

The system is decomposed into **four isolated ROS 2 nodes**, each with a single, well-defined responsibility. They communicate exclusively via ROS 2 topics and services, ensuring clean separation of concerns and independent testability.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Node Architecture                             │
│                                                                             │
│  ┌──────────────────────┐        ┌──────────────────────┐                   │
│  │   Mission Commander  │◄──────►│   Flight Controller  │                   │
│  │     (The Brain)      │  FSM   │     (The Muscle)     │                   │
│  │  Finite State Machine│ state  │  Visual Servoing &   │                   │
│  │  TAKEOFF→WANDERING   │        │  Velocity Commands   │                   │
│  │  →SEARCHING→LANDING  │        └──────────┬───────────┘                   │
│  │  →DWELLING           │                   │ /cmd_vel                      │
│  └──────────────────────┘                   ▼                               │
│                                   ┌─────────────────┐                       │
│  ┌──────────────────────┐         │  Gazebo Sim /   │                       │
│  │   Target Detector    │────────►│  ros_gz_bridge  │                       │
│  │     (The Eyes)       │  HUD    │                 │                       │
│  │  OpenCV HSV Masking  │         └─────────────────┘                       │
│  │  Centroid Detection  │                                                   │
│  │  Live HUD Overlay    │                                                   │
│  └──────────────────────┘                                                   │
│                                                                             │
│  ┌──────────────────────┐                                                   │
│  │   Base Teleporter    │                                                   │
│  │  (Command & Control) │                                                   │
│  │  Multi-threaded CLI  │                                                   │
│  │  JSON UserCommands   │                                                   │
│  └──────────────────────┘                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

### Node 1 — Mission Commander *(The Brain)*

The central orchestrator of the entire flight stack. It implements a **strict Finite State Machine (FSM)** that governs all high-level mission logic through five mutually exclusive operational modes:

| State | Description |
|---|---|
| `TAKEOFF` | Commands the drone to climb to a safe operational altitude before any mission logic executes. |
| `WANDERING` | Executes a time-boxed interior sweep pattern to gain spatial awareness of the environment. Altitude-locked; no target acquisition occurs in this state. |
| `SEARCHING` | Activates the target detection pipeline and transitions the drone into an **Archimedean spiral search algorithm** to systematically cover the 10×10 m geofence. |
| `LANDING` | Engages the visual servoing controller upon target detection. The FSM locks the drone into a precision descent profile guided by real-time pixel-error feedback. |
| `DWELLING` | Confirms a stable, centred touchdown via a precision timer. Holds position until the landing mission is officially declared complete. |

**Key design principles:**
- **Altitude-locked state isolation** — state transitions are only permitted when the drone satisfies strict altitude thresholds, preventing mid-maneuver mode switches.
- **Precise internal timers** — each timed state (Wandering, Dwelling) uses ROS 2 wall timers for deterministic, repeatable behaviour independent of simulation speed.

---

### Node 2 — Flight Controller *(The Muscle)*

Translates high-level FSM commands and visual feedback into low-level `geometry_msgs/Twist` velocity commands published to the drone. Key algorithms include:

- **Visual Servoing** — Computes X/Y/Z body-frame velocities directly from the pixel-space error between the detected target centroid and the image centre. Proportional gains are tuned to ensure smooth, oscillation-free convergence.
- **Sine-Wave Interior Sweep** — During `WANDERING`, the drone executes a smooth sinusoidal lateral sweep to maximise sensor coverage within the bounded area.
- **Archimedean Spiral Search** — During `SEARCHING`, an outward-expanding spiral ensures exhaustive, non-redundant coverage of the geofence until the target is acquired.
- **High-Altitude Deadbands** — Velocity commands are suppressed within configurable deadband thresholds at altitude to prevent oscillatory corrections from small sensor noise.
- **2.5-Second "Deep Escape" Logic** — Detects and resolves boundary-chattering conditions on the 10×10 m geofence by issuing a sustained counter-velocity command, preventing the drone from becoming trapped oscillating along a boundary.

---

### Node 3 — Target Detector *(The Eyes)*

A fully self-contained **OpenCV computer vision pipeline** that processes raw camera feeds from the Gazebo simulation:

- Ingests raw image data from the simulated downward-facing camera via the `ros_gz_bridge`.
- Applies **HSV colour-space masking** to robustly isolate the landing pad under varying simulated lighting conditions.
- Computes the **target centroid** from the largest detected contour and publishes its pixel coordinates.
- Renders and publishes a **dynamic, time-synchronised Heads-Up Display (HUD)** back into the Gazebo UI, overlaying bounding boxes, centroid markers, state annotations, and pixel-error vectors for real-time diagnostics.

---

### Node 4 — Base Teleporter *(Command & Control)*

An interactive Command & Control interface for dynamic mission management:

- Implements a **multi-threaded terminal interface** that accepts operator commands without blocking the ROS 2 spin loop.
- Utilises Gazebo's **`UserCommands` plugin** with strict, validated **JSON payloads** to teleport the landing pad to arbitrary positions within the geofence at runtime.
- **Actively blocks** relocation requests during critical flight phases (e.g., `LANDING`, `DWELLING`) and paused simulation states, preventing unsafe mid-maneuver target displacement.
- Provides real-time feedback to the operator on command acceptance, rejection, and current FSM state.

---

## Prerequisites

The following software must be installed on your system before building this project.

| Dependency | Version | Notes |
|---|---|---|
| **Ubuntu** | 22.04 LTS (Jammy Jellyfish) | Recommended and tested platform |
| **ROS 2** | Humble Hawksbill | [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| **Gazebo** | Harmonic (formerly Ignition) | [Installation Guide](https://gazebosim.org/docs/harmonic/install_ubuntu/) |
| **ros_gz_bridge** | Compatible with your ROS 2 / Gazebo versions | Bridges ROS 2 topics to Gazebo transport |
| **OpenCV** | 4.x | `sudo apt install python3-opencv` |
| **colcon** | Latest | `sudo apt install python3-colcon-common-extensions` |
| **rosdep** | Latest | `sudo apt install python3-rosdep` |

---

## Installation & Build

### Step 1 — Source your ROS 2 installation

```bash
source /opt/ros/humble/setup.bash
```

To make this permanent across all terminal sessions:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2 — Create a colcon workspace and clone the repository

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/DineshMoorthy007/Drone-State-Navigation-Sim.git
```

### Step 3 — Install ROS dependencies with rosdep

```bash
cd ~/drone_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4 — Build the workspace

```bash
cd ~/drone_ws
colcon build --symlink-install
```

### Step 5 — Source the workspace overlay

```bash
source ~/drone_ws/install/setup.bash
```

Append to your shell configuration to avoid repeating this in every new terminal:

```bash
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Note:** Always ensure both the global ROS 2 setup and the workspace overlay are sourced before executing any `ros2` commands.

---

## Usage / How to Run

### Launch the Full Simulation Stack

This command starts the Gazebo world, spawns the drone model, and brings up all four ROS 2 nodes (Mission Commander, Flight Controller, Target Detector) in a single coordinated launch:

```bash
ros2 launch auto_landing auto_landing.launch.py
```

Once launched, the simulation will proceed autonomously through the full FSM sequence: **TAKEOFF → WANDERING → SEARCHING → LANDING → DWELLING**.

---

### Start the Command & Control Teleporter

In a **separate terminal** (with the workspace sourced), launch the interactive Base Teleporter to dynamically reposition the landing pad during the mission:

```bash
ros2 run auto_landing base_teleporter
```

The terminal will display an interactive prompt. Follow the on-screen instructions to enter target coordinates. The teleporter will automatically reject commands during critical flight phases (`LANDING`, `DWELLING`) to ensure flight safety.

---

> **Tip:** Open additional terminals to inspect the live ROS 2 topic graph and monitor system state:
> ```bash
> # Monitor the FSM state in real time
> ros2 topic echo /drone/state
>
> # Inspect the full topic graph
> ros2 topic list
> ```
