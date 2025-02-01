# Leader Follower Network in a Closed Environment

## Overview
This project is a real-robot implementation of distributed control systems in a multi-robot disinfection operation. It was developed as part of the **ENME 808T: Network Control Systems** course at the University of Maryland, Fall 2024.

The project simulates a scenario where autonomous robots navigate through a clinical setting to disinfect high-traffic areas while maintaining network connectivity and avoiding obstacles. The system operates under decentralized control strategies to ensure robustness and efficiency.

## Project Details
### Mission Objectives
The project consists of four sub-missions:
1. **Initial Deployment**: Robots navigate from the refueling station to the target disinfection area while maintaining connectivity and avoiding obstacles.
2. **Room Disinfection**: The robots cover a defined area while ensuring a majority (>85%) of the space is disinfected.
3. **Return to Refueling Station**: The robots return to the station while surveying the environment for future disinfection planning.
4. **Refueling**: The robots arrange themselves in a predefined formation to refuel.

Each sub-mission involves designing a **formation graph topology** and **decentralized control strategies** to achieve the objectives.

## Implementation
The project was implemented using:
- **MATLAB** for simulation and control design.
- **Robotarium** for real-world implementation.
- **Distributed control algorithms** including formation control, obstacle avoidance, and Voronoi-based coverage control.

## Setup
Download both the folders into your workspace
- Step 1: Run init.m file in **Robotarium-Simulator**, this initializes the robotarium setup in the workspace.
- Step 2: Run main.m in **ENME808T_Final_Project**, you can change the mission at start of the file.
[📽️ Watch the video](https://github.com/akoushik2k/swarm_control-Network-Control-Systems/blob/main/Final%20Video%20Robotarium.mp4)
