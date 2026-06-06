# Loquro Project

This repository contains the software and simulation environment for the **Loquro** balancing robot.

## Project Structure

The project is organized as a modular Python workspace, with the core logic residing in the `src` directory.

### Loquro Lab (`src/loquro_lab/`)

`loquro_lab` is a Python package managed with **uv**. It serves as the main research and development sandbox for the robot's control and physics.

**Key components included in this package:**

*   **MuJoCo Simulation:** A high-fidelity physics environment to test the robot's balancing and movement.
*   **Robot Assets & XMLs:** All structural definitions, meshes, and MJCF (MuJoCo) configuration files.
*   **Training & Models:** Scripts for training control policies and ready-to-use **ONNX** models for inference.
*   **Keyboard Demo:** An interactive script to manually control the robot within the simulation using your keyboard.

## Getting Started

To work with the simulation and training tools, you will need the [uv](https://github.com/astral-sh/uv) package manager installed.

For detailed information on installation, environment setup, and how to run the simulation or the demos, please refer to the specific documentation inside the package folder:

👉 **[Read the loquro_lab documentation here](./src/loquro_lab/README.md)**