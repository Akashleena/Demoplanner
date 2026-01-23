# Constrained Motion Planning for Warehouse Pick-and-Place

Multi-start RRT planning with orientation constraints for top-down grasping in warehouse environments.  
Built with MoveIt 2 and ROS 2 Jazzy.

---
## Demo
[View the demo presentation](https://docs.google.com/presentation/d/1olo1cGZQv2Bil8W0DPs7fW4IPLR5U7UeDcmfDIqGfHU/edit?usp=sharing)


## Overview

Warehouse pick-and-place tasks often require the end-effector to remain top-down throughout the motion.  
However, standard sampling-based planners (e.g., RRT) struggle when strict orientation constraints are imposed.

This project empirically studies the practical limits of constrained RRT planning and demonstrates how multi-start strategies improve reliability—up to a point.

---


## Problem Statement

- RRT samples randomly in joint space
- Orientation constraints define a lower-dimensional manifold
- As constraint tolerance tightens, valid samples become vanishingly rare
- Planning becomes slow, unstable, or fails entirely

---

## Approach

1. Apply an orientation constraint (top-down gripper) using MoveIt’s constraint API  
2. Run multiple independent RRT attempts per query  
3. Score candidate trajectories and select the best result  
4. Sweep orientation tolerance values and log performance metrics

---

## Key Findings

| Orientation Tolerance | Success Rate | Avg Planning Time | Path Quality |
|----------------------|--------------|-------------------|--------------|
| ±23° | 100% | ~60 ms | Optimal (~2 rad) |
| ±10° | 100% | ~200 ms | Optimal |
| ±5°  | 100% | ~560 ms | Optimal |
| ±2°  | 100% | ~10 s | Degraded (~7 rad) |
| ±1°  | 50%  | ~15 s | Erratic detours |
| ±0.5°| 0%   | Timeout | — |

Takeaway:  
Sampling-based planners like RRT have a practical orientation constraint limit of ~±2–5°.  
Below this range, Cartesian or hybrid planners are more appropriate.

---

## Features Implemented

### Feature 1: Constrained Multi-Start RRT Planning
- Multi-start RRT planning (K = 6 attempts)
- End-effector orientation constraint (top-down grasping)
- Trajectory scoring based on:
  - Path length
  - Smoothness
- Constraint tolerance sweep with CSV logging
- Warehouse-style scene with shelf obstacles

### Feature 2: Hybrid Pick-and-Place Planning (Task-Aware)
Industrial pick-and-place requires different planning strategies for different motion phases.  
This project implements a **hybrid task-space + sampling-based planning structure**:

| Phase | Planner | Why |
|------|--------|-----|
| Transit (home → prepick) | RRT | Obstacle avoidance, any valid path |
| Approach (prepick → pick) | Cartesian | Straight-line, predictable |
| Retreat (pick → postpick) | Cartesian | Prevent object drag / collisions |
| Transfer (postpick → preplace) | RRT | Navigate around obstacles |
| Approach (preplace → place) | Cartesian | Precise placement |

#### Task-Space Frame Generator
Inspired by Russ Tedrake’s manipulation course, the system generates *semantic task frames* from object poses:

---

## Running the Demo
For Feature 1
```bash
# Terminal 1: Start MoveIt demo
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Terminal 2: Load warehouse scene
ros2 run warehouse_demo_py plan_warehouse_pick

# Terminal 3: Run planning experiments
ros2 run warehouse_pick_cpp warehouse_systems_demo
```
For Feature 2
### Run Pick-and-Place Demo
```bash
# Terminal 1: MoveIt
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Terminal 2: Scene
ros2 run warehouse_demo_py warehouse_picker

# Terminal 3: Pick-and-place
ros2 run warehouse_pick_cpp warehouse_pick_place
```
## Next Steps

- Dense collision validation  
  To catch collisions missed by waypoint-only checking near shelves and thin obstacles.

- Trajectory caching  
  To reduce planning latency for repeated warehouse picks with similar start and goal states.

- Manipulability and singularity awareness  
  To avoid near-singular configurations that often appear in constrained motion.

- Adaptive constraint relaxation  
  To automatically detect infeasible constraints and relax them safely instead of timing out.
- ICP Based Pose estimation, Grasp failure recovery and dynamic obstacles
- Once we achiieve all this lets move to mujoco or nvidia issac sim to do some trendy super sexy physical AI stuff xD (joking nope fundamentals first hype later. I am old school.)

