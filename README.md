<div align="center">
  
  <h1>LoQuRo</h1>
  <p><b>A fully custom-designed, AI-controlled quadrupedal robot built for under €300.</b><br>
  <i>Proof that autonomous legged locomotion doesn't require a six-figure budget.</i></p>

  ![Mechatronics](https://img.shields.io/badge/Mechatronics-c8b89a?style=for-the-badge)
  ![Deep RL](https://img.shields.io/badge/Deep_RL-888580?style=for-the-badge)
  ![ROS 2](https://img.shields.io/badge/ROS_2-e25c3b?style=for-the-badge)
  ![Open Source](https://img.shields.io/badge/Open_Architecture-161616?style=for-the-badge&logo=github&logoColor=white)

  <br><br>

  <video controls width="100%" style="max-width: 600px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.1);">
    <source src="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/raw/main/docs/images/promotion.mp4" type="video/mp4">
  </video>
  <br>
  <a href="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/raw/main/docs/images/promotion.mp4">Watch promotional video</a>

</div>

---

## The Market Gap & Opportunity

> [!NOTE]
> **The Paradigm Shift:** Advanced quadrupedal platforms currently retail at €1,600–€3,000, operating as closed, black-box systems that limit R&D scalability. Low-cost DIY alternatives fail to provide the torque feedback and bandwidth required for modern AI deployment. LoQuRo bridges this gap with a highly optimized, open-architecture platform that drastically lowers the barrier to entry for reinforcement learning research and commercial deployment.

| Metric | Value | Strategic Advantage |
| --- | --- | --- |
| **BOM Cost** | `€278` | < 25% of entry-level commercial alternatives; highly scalable for fleets. |
| **DOF** | `12` | 3 per leg (full hip + knee articulation) enabling complex gait maneuvers. |
| **Weight** | `1.8 kg` | Exceptionally lightweight, reducing power consumption and material costs. |
| **Inertia** | `x13 Reduction` | Patented-style mechanical innovation maximizing commodity hardware efficiency. |

---

## Core Engineering Innovations

<details>
<summary><b>[01] Four-bar linkage leg mechanism</b></summary>
<br>
Rather than mounting the knee actuator directly at the joint, LoQuRo relocates it to the chassis via an aluminium four-bar transmission. This drastically reduces the rotational inertia of each leg by a factor of 13, enabling low-cost commodity servos to deliver dynamic performance margins usually reserved for expensive proprietary actuators.
<br><br>
<b>Repository link:</b> <a href="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Mechanics">View Mechanics R&D</a>
</details>

<details>
<summary><b>[02] GPU-parallelized PPO training</b></summary>
<br>
The locomotion policy was trained entirely in a digital twin using MuJoCo and mjlab, executing thousands of parallel environments simultaneously on GPU. The resulting ONNX model is highly efficient, running real-time inference on edge devices like a laptop or Raspberry Pi.
<br><br>
<b>Repository link:</b> <a href="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Software">View AI & Control Source</a>
</details>

<details>
<summary><b>[03] Advanced Embedded Firmware & Decentralized Control</b></summary>
<br>
A significant portion of the system's complexity resides in its firmware architecture. Bridging AI and hardware required writing highly optimized embedded C++ for the ESP32-S3. Leveraging FreeRTOS for deterministic multicore execution and Eclipse Zenoh (a lightweight data-centric middleware), the firmware achieves a blistering 7–9 ms WiFi latency. This ensures the 50 Hz real-time control loop remains perfectly synchronized for reactive locomotion.
<br><br>
<b>Repository link:</b> <a href="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Software">View Firmware Source</a>
</details>

<details>
<summary><b>[04] Custom dual-PCB architecture</b></summary>
<br>
Two purpose-designed, stackable PCBs handle per-leg power distribution, high-speed I²C multiplexing across 12 magnetic encoders, and IMU sensor fusion. By integrating everything at the board level, we eliminated wiring complexity, mitigated failure points, and paved the way for mass-manufacturing scalability.
<br><br>
<b>Repository link:</b> <a href="https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Electronics">View Electronics & PCB Design</a>
</details>

---

## Development Outcomes & Viability

The core architecture is proven. The sim-to-real pipeline successfully transfers digital training to physical hardware.

* **[ 100% ] Stable walking policy trained in simulation**
PPO agent successfully learned forward, backward, and rotational gait patterns. Policy converged efficiently and exported to ONNX for edge deployment.
* **[ 100% ] Full-stack hardware & firmware integration**
The complex interplay between the embedded ESP32 firmware, custom PCBs, ROS 2 inference node, and Zenoh link was validated end-to-end with zero bottlenecks.
* **[ 85% ] Autonomous locomotion under strict budget**
Physical walking was achieved during outdoor testing. A transient current spike exceeded the XL4015 buck converters' limits, resulting in a servo failure. The root cause is fully diagnosed and serves as a vital stress-test data point.
* **[ Next Phase ] Iteration 2 for Commercial Readiness**
Transitioning from prototype to robust product. Planned upgrades include XL4016 modules, a 4-layer PCB with dedicated power planes for electrical resilience, and optimized structural geometry.

> [!WARNING]
> **Investment Context:** The hardware interruption during extreme load testing is a known boundary condition of the €300 constraint, not a fundamental design flaw. The control logic, firmware, and mechanical IP are completely sound. An allocation of just €150 in targeted component upgrades permanently resolves this bottleneck for version 2.0.

---

## Repository & Technical Assets

The architecture is cleanly separated into distinct, highly documented modules:

* **[Control & AI](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Software):** Sim-to-real pipelines, MuJoCo digital twins, PPO training scripts, and ROS 2 high-level nodes.
* **[Firmware](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Software):** The critical embedded layer. Highly optimized FreeRTOS multicore routines on ESP32-S3, Zenoh DDS bridging, and low-latency I²C sensor polling.
* **[Mechanics](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Mechanics):** PTC Creo CAD source files, STP assemblies, and URDF/MJCF simulation meshes.
* **[Electronics](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/tree/main/Electronics):** EasyEDA schematics, stackable PCB layouts, and comprehensive BOM.

**Quick Links:**
[Project Website](https://www.google.com/search?q=https://integrated-project-2-2026-uvic-ucc.github.io/LoQuRo/) | [Technical PDF](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo/raw/main/docs/Technical_doc_IP2.pdf) | [GitHub Repository](https://www.google.com/search?q=https://github.com/Integrated-Project-2-2026-UVic-UCC/LoQuRo)

---

*LQR · LoQuRo · Lowcost Quadruped Robot · Scalable Mechatronics · UVic–UCC · June 2026*
