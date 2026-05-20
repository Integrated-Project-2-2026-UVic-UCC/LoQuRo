<!-- HEADER & BANNER -->
<div align="center">
  
  <br>
  <h1>🐕 LoQuRo</h1>
  <p>
    <b>Design. Build. Train.</b> <br>
    A Mechatronics Engineering project exploring Reinforcement Learning in Robotics.
  </p>

  <!-- BADGES (Escudos decorativos) -->
  <img src="https://img.shields.io/badge/Status-Defining-yellow?style=for-the-badge" alt="Status">
  <img src="https://img.shields.io/badge/Area-Mechatronics-blue?style=for-the-badge" alt="Mechatronics">
  <img src="https://img.shields.io/badge/AI-Reinforcement%20Learning-green?style=for-the-badge" alt="AI">
  <br><br>
</div>

---

<!-- INTRODUCTION -->
## 📖 Introduction

Nowadays, robots are gaining relevance due to their capacity to simulate and improve complex tasks. Significant effort is currently being directed toward **self-learning capabilities** in autonomous systems.

Our proposal is the design and creation of a **Quadrupedal Robot controlled via Reinforcement Learning (RL)**. The goal is to simulate canine behavior and optimize locomotion through trial and error, stepping out of our comfort zone to tackle new challenges.

> **The Team:** We are a group of four students experienced in different areas of **mechatronics** and robotics competitions.

---

<!-- OBJECTIVES SECTION (Grid Layout) -->
## 🎯 Objectives

The main objective is to **design, build, and train** a quadrupedal robot capable of autonomous locomotion using AI agents.

### Specific Goals

| Feature | Description |
| :--- | :--- |
| 🔋 **Autonomy** | Transformer alimentation for continous power supply during operation. |
| ⚖️ **Balance** | Robust dynamic balance control to handle uneven surfaces. |
| 📦 **Design** | Compact, lightweight, yet structurally sound mechanical design. |
| 🛡️ **Durability** | High robustness to withstand the physical stress of learning (falling and recovering). |

---

<!-- FOLDER STRUCTURE (The Core) -->
## 📂 Repository Structure

This project follows a multidisciplinary approach, divided into three main pillars:

<div align="center">

### 1. 🛠️ Mechanics
**`./Mechanics`**
</div>

> Contains all the hardware design files necessary to build the physical robot.
> *   **CAD Models:** `PTC Creo` source files.
> *   **3D Printing:** `.STP` files ready for slicing.
> *   **Simulation:** URDF/Meshes for simulation environments (MuJoCo).

<br>

<div align="center">

### 2. ⚡ Electronics
**`./Electronics`**
</div>

> Houses the nervous system of the robot.
> *   **Schematics:** Wiring diagrams and connection maps.
> *   **PCB Design:** EasyEDA files for custom boards.
> *   **BOM:** Bill of Materials (Sensors, Actuators, Microcontrollers, Batteries).

<br>

<div align="center">

### 3. 💻 Software
**`./Software`**
</div>

> The brain of the project, split into low-level control and high-level AI.
> *   **Firmware:** Code for the microcontrollers (motor drivers, IMU data reading). Comunication with control via Wifi. Muticore task management.
> *   **Reinforcement Learning:** MuJoCo training environments, generated control models.
> *   **Control:** ROS2 packeges, inverse kinematics and gait planning algorithms.

---

<!-- FOOTER -->
<div align="center">
  <br>
  <p>Made by Team 1</p>
  <p>
    <a href="#">Documentation</a> •
    <a href="#">Video Demo</a> •
    <a href="#">Contact</a>
  </p>
</div>
