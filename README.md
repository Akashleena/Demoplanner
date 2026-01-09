```md
# Constraint-Aware Motion Planning with Multi-Start RRT and Trajectory Optimization

Production-style manipulation planning for MoveIt 2 that addresses real-world deployment challenges: probabilistic planner variance, explicit collision safety, execution quality, and task repeatability.

## Key Features

Multi-Start Planning – Runs 6 sequential RRT attempts, scores candidates by path length + smoothness + collision validity, selects best trajectory

CHOMP-lite Smoothing – Post-processing optimization (10-20 iterations) reduces curvature while preserving collision-free property and constraint satisfaction

Dense Collision Validation – Explicit interpolation between waypoints catches collisions that discrete checking misses; adaptive resolution based on proximity to obstacles

Trajectory Caching – Keyed by (start state, goal pose, constraints, scene hash). Cold run plans and caches; warm run retrieves in milliseconds

Path Constraints – End-effector upright orientation enforced via MoveIt's constraint API throughout planning

## Demo Setup
Scene: Shelf obstacles + target object  
Task: Constrained reach-to-pose with Panda arm  
Visual Output: RViz overlay of raw RRT (blue) vs smoothed (green) trajectories

Console Output:
```
Attempt 0 | valid=1 | dense=1 | score=12.34
Attempt 1 | valid=1 | dense=0 | score=10.02
Best: attempt 1
[CACHE STORE]

[Run 2]
[CACHE HIT] 6ms
```
## Run Instructions

```bash
# Terminal 1
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Terminal 2
ros2 run warehouse_demo_py plan_warehouse_pick

# Terminal 3
ros2 run warehouse_pick_cpp warehouse_systems_demo
```


## Why It Matters

Demonstrates systems-level reasoning for production robotics:
- Handling probabilistic planner behavior
- Balancing safety and efficiency 
- Optimizing for repeated warehouse-style tasks
- Engineering collision-safe motion beyond default MoveIt guarantees

