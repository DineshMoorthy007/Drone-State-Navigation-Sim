# Autonomous Drone Landing System

A 3D simulation environment that demonstrates a fully autonomous drone tracking and landing precisely on the **center of a moving base**. Built with **ROS2** and **Gazebo**, the system employs a strict four-state machine — Wandering, Searching, Landing, and Dwelling — combined with precision control algorithms to achieve stable, repeatable autonomous landings in dynamic environments.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [System Architecture](#system-architecture)
- [Future Improvements](#future-improvements)

---

## Prerequisites

Ensure the following software is installed on your system before proceeding:

- **ROS2** (Humble Hawksbill or later recommended)
  - [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo** (Classic 11 or Gazebo Ignition, compatible with your ROS2 distribution)
  - [Gazebo Installation Guide](https://gazebosim.org/docs)
- **colcon** build tool
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- **Python 3.8+** (included with most ROS2 distributions)
- **Git**
  ```bash
  sudo apt install git
  ```
- Standard ROS2 dependencies (e.g., `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`)

---

## Installation

1. **Clone the repository** into your ROS2 workspace source directory:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/DineshMoorthy007/Drone-State-Navigation-Sim.git
   ```

2. **Install ROS2 package dependencies** using `rosdep`:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

4. **Source the workspace** overlay:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## Usage

Launch the full simulation (Gazebo environment + drone state machine):

```bash
ros2 launch drone_landing_sim drone_landing.launch.py
```

To monitor the current state machine mode in real time:

```bash
ros2 topic echo /drone/state
```

To visualize the drone's pose and trajectory in RViz:

```bash
rviz2 -d ~/ros2_ws/src/<your-repo-directory>/config/drone_viz.rviz
```

> Replace `<your-repo-directory>` with the name of the directory you cloned the repository into (default: `Drone-State-Navigation-Sim`).

> **Note:** Always source the workspace (`source ~/ros2_ws/install/setup.bash`) in every new terminal session before running any ROS2 commands.

---

## System Architecture

The core of the system is a **strict four-state machine**. Each state has a well-defined entry condition, behaviour, and exit condition. The drone transitions through states sequentially based on sensor feedback and internal timers.

```
┌──────────────┐     Stable altitude reached     ┌──────────────┐
│  WANDERING   │ ──────────────────────────────► │  SEARCHING   │
│   (30 s)     │                                  │              │
└──────────────┘                                  └──────┬───────┘
                                                         │ Base detected
                                                         ▼
                                                  ┌──────────────┐
                                                  │   LANDING    │
                                                  │              │
                                                  └──────┬───────┘
                                                         │ Centered on base
                                                         ▼
                                                  ┌──────────────┐
                                                  │   DWELLING   │
                                                  │   (5 s)      │
                                                  └──────────────┘
```

### State 1 — Wandering Mode

- **Trigger:** System start-up.
- **Behaviour:**
  - The drone executes a controlled **vertical take-off** to a predefined fixed altitude.
  - Once the target altitude is confirmed stable, a **30-second exploration timer** begins.
  - During this period the drone navigates the environment using a smooth, efficient coverage algorithm (e.g., lawnmower or spiral sweep) to build spatial awareness.
- **Key constraint:** The drone does **not** calculate the base position or initiate any landing logic during this mode. This ensures the drone is fully stabilised and has adequate altitude before committing to a search.
- **Exit condition:** The 30-second timer expires → transition to **Searching Mode**.

### State 2 — Searching Mode

- **Trigger:** Expiry of the Wandering Mode timer.
- **Behaviour:**
  - The drone activates its **base-detection pipeline** (camera + image processing / sensor fusion).
  - It manoeuvres systematically to maximise sensor coverage while continuously scanning for the moving base.
  - Heading and velocity commands are updated at each control cycle based on the latest sensor readings.
- **Exit condition:** The moving base is detected and its position is confirmed → transition to **Landing Mode**.

### State 3 — Landing Mode

- **Trigger:** Successful detection of the moving base.
- **Behaviour:**
  - The drone switches to a **precision approach controller**.
  - It continuously tracks the **real-time position of the moving base** and adjusts horizontal and vertical velocity to converge on the base's centre point.
  - A predictive or reactive control strategy compensates for base movement during the descent.
  - Descent rate is modulated to ensure the drone reaches the centre of the base — not merely anywhere on it.
- **Exit condition:** The drone's position matches the base centre within a defined tolerance and touchdown is confirmed → transition to **Dwelling Mode**.

### State 4 — Dwelling Mode

- **Trigger:** Confirmed touchdown on the centre of the moving base.
- **Behaviour:**
  - The drone holds its landed position while a **5-second confirmation timer** runs.
  - This dwell period validates that the landing was stable and not a transient contact.
  - All flight controllers are kept active at low power to counteract any residual disturbances from base motion.
- **Exit condition:** The 5-second timer expires → landing mission **complete**.

---

## Future Improvements

- **Multi-drone coordination:** Extend the state machine to support swarm scenarios where multiple drones take off, search, and land on separate moving bases simultaneously.
- **Dynamic obstacle avoidance:** Integrate real-time LiDAR-based collision avoidance during Wandering and Searching modes.
- **Adaptive landing controller:** Replace the fixed-gain descent controller with a model-predictive controller (MPC) to improve robustness against faster or less predictable base motion.
- **Real hardware deployment:** Port the ROS2 nodes to a physical drone platform (e.g., PX4 / ArduPilot via MAVROS2) for hardware-in-the-loop (HIL) and real-world testing.
- **Machine learning integration:** Train a vision-based policy (e.g., using reinforcement learning) to replace the hand-crafted search pattern with a learned, more efficient exploration strategy.
- **Ground truth benchmarking:** Add automated test scenarios in Gazebo to record landing precision metrics (centre offset, landing time, success rate) across varied base speeds and trajectories.
