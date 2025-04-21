# 🛩️ Sim2Real PX4 AI Drone Framework

> A modular simulation-to-reality framework for AI-based drone training, data collection, and manual control using PX4, ROS 2, MAVSDK, and Gazebo.

---

## 📦 Overview

**Sim2Real** is a PX4 drone development environment supporting:
- ✅ **Manual keyboard control** via MAVSDK & GUI
- 🎓 **Reinforcement Learning** with concurrent training during simulation
- 🎯 **Autonomous flight testing** with pre-trained models
- 📸 **Real-time camera bridging** from Gazebo to ROS2
- 🧠 Easily extendable to your own AI pipeline or control logic

---

## 🛠️ Components

| File / Folder       | Description                                           |
|---------------------|-------------------------------------------------------|
| `.env`              | Centralized environment configuration                 |
| `run.sh`            | Task launcher (monitor, collect, train, test, etc.)   |
| `setup.sh`          | One-line installation for PX4, ROS2, and dependencies |
| `monitor_run.py`    | GUI-based manual control interface (via MAVSDK)       |
| `src/`              | Your AI controller, data pipeline, reward module etc. |
| `models/`           | Trained models (e.g., SAC checkpoints)                |
| `logs/`             | Flight data and training logs                         |

---

## 🚀 Quickstart

### ✅ 1. Install

```bash
bash setup.sh
```
✅ 2. Launch Simulation Environment
```bash
./run.sh --monitor
```
This starts PX4 SITL, Gazebo + Gimbal, MAVROS, ROSBridge, image viewer, and XRCE agent in tmux.

✅ 3. Manual Keyboard Control (for validation)
```bash
pdm run python monitor_run.py
```
✅ 4. Live Training with AI
```bash
./run.sh --train-live   # Collect and Training (to be continued...)
```
Or other modes:
```bash
./run.sh --collect      # Collect flight data (to be continued...)
./run.sh --train        # Offline training (to be continued...)
./run.sh --test         # Model testing (to be continued...)
```
