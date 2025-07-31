# Isaac-sim-go2-with-arm

**NVIDIA Isaac Sim 4.5 + ROS 2 whole-body PD control for Go2 quadruped with arm.**  
Features stable crawling, trotting, and fully tunable gaits with real-time joint-space feedback using ROS 2 topics. Modular Python code, ready for research and education in simulation-based legged robotics.

---

## 🦿 Project Overview

This project enables robust **whole-body joint-space control** of a Go2-style quadruped robot with a 7-DoF manipulator arm in NVIDIA Isaac Sim 4.5, fully controlled from ROS 2 (Humble).  
The system implements:

- Real-time feedback and control via `/joint_states` and `/robot/joint_commands`
- Multiple gaits: **ultra-stable crawl/walk** (three legs always down) and classic **trot** (diagonal pairs)
- Python ROS 2 nodes for **PD control** and gait generation
- Full compatibility with Isaac Sim's Action Graph and Articulation Controller

---

## ✨ Features

- **Action Graph bridge** between Isaac Sim and ROS 2:  
  - Publishes `/joint_states`
  - Subscribes to `/robot/joint_commands`
- **PD Controller** for robust joint tracking and smooth, feedback-based motion
- **Gait modules:**  
  - `walk_crawl_pd_controller.py`: Ultra-stable crawl (three legs in stance)
  - `trot_pd_controller.py`: Classic trotting
- **Easy parameter tuning** for frequency, amplitude, offsets, and controller gains
- **Full-body support:** All 12 leg joints + 7-DoF arm

---

## 📁 Directory Structure

go2_ws/
├── src/
│ └── go2_trot_publisher/
│ ├── go2_trot_publisher/
│ │ ├── trot_publisher.py
│ │ ├── trot_pd_controller.py
│ │ └── walk_crawl_pd_controller.py
│ ├── package.xml
│ └── setup.py
├── urdf/ # (your robot URDFs)
├── meshes/ # (your robot/arm meshes)
├── LICENSE
├── README.md
└── ...


---

## 🚀 Getting Started

### Prerequisites

- **Ubuntu 22.04**
- **Isaac Sim 4.5** (with ROS 2 bridge enabled)
- **ROS 2 Humble**
- Python 3.10+, numpy

### Isaac Sim Setup

1. Launch Isaac Sim and load your Go2 robot + arm.
2. Open the **Action Graph** and set up:
    - `On Playback Tick` node
    - `ROS2 Publish Joint State` (to `/joint_states`, targeting robot root prim)
    - `ROS2 Subscriber` (to `/robot/joint_commands`)
    - `Articulation Controller` for your robot
    - Wire nodes so only **one command source** is active at a time

### ROS 2 Workspace

1. Clone or unzip this workspace:
    ```bash
    cd ~
    # git clone https://github.com/nayon007/Isaac-sim-go2-with-arm.git
    cd Isaac-sim-go2-with-arm/go2_ws
    ```
2. Build and source:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## 🏃 Usage

### **Ultra-stable crawl/walk (three feet down)**
```bash
ros2 run go2_trot_publisher walk_crawl_pd_controller
```
### **Classic diagonal trot
```bash
ros2 run go2_trot_publisher trot_pd_controller
```

## ⚙️ Parameter Tuning
Edit the Python scripts in go2_trot_publisher/:

freq: gait frequency (lower = slower, more stable)

thigh_offset, calf_offset: more upright for stability

thigh_amp, calf_amp: reduce for less swing

Kp: PD gain (lower for softer, higher for stiffer)


## 🧠 How It Works
Isaac Sim Action Graph bridges /joint_states and /robot/joint_commands

ROS 2 Python node generates gait joint references (walk or trot)

PD controller tracks reference using feedback from /joint_states

Articulation Controller executes the position command in Isaac Sim

## 🩹 Troubleshooting
Robot moving backward: Flip the sign of thigh_amp and calf_amp or adjust phase.

Unstable? Lower frequency and amplitude; increase offsets.

Legs too crouched? Increase thigh_offset and/or decrease (more negative) calf_offset.

No movement? Check topic names, Action Graph, and PD gain.

## 📚 Credits & License
Based on NVIDIA Isaac Sim, ROS 2, and the open-source robotics community.

Author: [Your Name/Handle]

MIT License (see LICENSE).
