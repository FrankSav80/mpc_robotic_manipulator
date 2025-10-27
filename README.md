# 🤖 Predictive Control of a Robotic Manipulator (Dynamic Obstacles)

Implemented a predictive control framework for a robotic arm capable of safe trajectory tracking in dynamic environments. Developed as **Master’s thesis** in Automation Engineering at Politecnico di Bari.

---

## 🚀 Overview

This project addresses the challenge of controlling an advanced robotic manipulator (Niryo Ned2) in environments where **dynamic obstacles** may interfere with task execution. 

<p align="center">
  <img src="images/robot_manipulator/niryo_ned2_joints.jpg" alt="Niryo Ned2 Joints" width="300">
</p>

The controller computes **optimal joint velocities** in real-time, integrating:

- **Kinematic prediction models** of the robot  
- A **Generalized Proportional Integral Observer (GPIO)** for estimating obstacle motion  
- **Safety constraints** based on Control Barrier Functions (CBFs) with **dynamically optimized decay rates**  

This approach ensures a **balance between efficiency and robustness**, adapting the safety margins in real-time while maintaining accurate trajectory tracking.

Simulation and testing are performed in **RViz** within the **ROS2 and MoveIt2** frameworks.

<p align="center">
  <img src="images/block_diagram.png" alt="System architecture" width="800">
</p>

---

## 🧰 Technologies

- ![C++](https://img.shields.io/badge/C%2B%2B-Programming%20Language-blue?style=flat-square&logo=c%2B%2B)
- ![Python](https://img.shields.io/badge/Python-Programming%20Language-yellow?style=flat-square&logo=python)
- ![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)
- ![MoveIt2](https://img.shields.io/badge/MoveIt-2-9cf?style=flat-square&logo=ros)
- ![CasADi](https://img.shields.io/badge/CasADi-Symbolic-orange?style=flat-square)
- ![IPOPT](https://img.shields.io/badge/Solver-IPOPT-lightgrey?style=flat-square)
- ![Linux](https://img.shields.io/badge/Linux-WSL2%20%7C%20Ubuntu-black?style=flat-square&logo=linux)

---

## ⚙️ System Description

The **closed-loop MPC controller** performs the following at each cycle:

1. Updates the robot's **current state**  
2. Computes the **Jacobian** once per cycle  
3. Builds and solves the **optimization problem** using **CasADi/IPOPT**  
4. Applies **warm-starting** with the previous solution  
5. Publishes the **optimal joint velocities** for execution

<p align="center">
  <img src="images/mpc/mpc_logic.png" alt="MPC Logic" width="500">
</p>

### Safety via CBF

- Ensures a **minimum safe distance** from obstacles  
- Safety constraints included as **decision variables** within the MPC problem  
- Dynamically optimized decay rates allow **real-time adaptation of safety margins**

<p align="center">
  <img src="images/cbf/cbf_advantages.PNG" alt="CBF Advantages" width="500">
</p>

### Dynamic Obstacles

- **GPIO observer** estimates the motion of obstacles in real-time  
- Enables predictive trajectory adjustment while maintaining safety  
- MPC adapts the robot motion to avoid collisions dynamically

<p align="center">
  <img src="images/safe_distance.png" alt="safe distance" width="500">
</p>

---

## 🧪 Results

- Accurate **trajectory tracking** in obstacle-free scenarios  
- Effective **collision avoidance** with moving obstacles  
- Stable and smooth motion of the Niryo Ned2 end-effector  
- Simulation results validate theoretical predictions

<p align="center">
  <img src="images/result.gif" alt="MPC simulation result" width="500">
</p>

---


## 📁 Repository Structure

📦 mpc_robotic_manipulator 
- src/
  - casadi_ws/casadi → CasADi symbolic and optimization files
  - matlab_script → MATLAB scripts
  - ned_ros → ROS2 configuration files for the Niryo Ned2 robot
  - niryo_moveit2_config → Main scripts developed for MPC implementation
- image/ → Figures, simulation results
- docs/ → Thesis report
- README.md → Project overview

---

## 👤 Author

Developed by [Francesco Savino](https://github.com/FrankSav80)  
Master’s Degree in Automation and Robotics Engineering – Politecnico di Bari

---

## 🧠 Keywords

`ROS2` • `MoveIt2` • `C++` • `Python` • `MPC` • `CBF` • `Dynamic Obstacles` • `Trajectory Tracking` • `Obstacle Avoidance`
