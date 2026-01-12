# Constrained Motion Planning for Warehouse Pick-and-Place

Multi-start RRT planning with orientation constraints for top-down grasping in warehouse environments.  
Built with MoveIt 2 and ROS 2 Jazzy.

---

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

- Multi-start RRT planning (K = 6 attempts)
- End-effector orientation constraint (top-down grasping)
- Trajectory scoring based on:
  - Path length  
  - Smoothness
- Constraint tolerance sweep with CSV logging
- Warehouse-style scene with shelf obstacles

---

## Running the Demo

```bash
# Terminal 1: Start MoveIt demo
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Terminal 2: Load warehouse scene
ros2 run warehouse_demo_py plan_warehouse_pick

# Terminal 3: Run planning experiments
ros2 run warehouse_pick_cpp warehouse_systems_demo
```
## Next Steps

- Dense collision validation  
  To catch collisions missed by waypoint-only checking near shelves and thin obstacles.

- Trajectory caching  
  To reduce planning latency for repeated warehouse picks with similar start and goal states.

- Hybrid Cartesian + RRT planning  
  To handle very tight orientation constraints where sampling-based planners become unreliable.

- Manipulability and singularity awareness  
  To avoid near-singular configurations that often appear in constrained motion.

- Adaptive constraint relaxation  
  To automatically detect infeasible constraints and relax them safely instead of timing out.

