# CFS-ROS

A ROS package implementing Convex Feasible Set (CFS) and Slack Convex Feasible Set (SCFS) Algorithm for motion planning.

## Features

- Hybrid Sampling and Optimization-based Planner
- Supports arbitrary convex collision meshes
- GPU accelerated minimum distance computation
- Critical kinematics and dynamics algorithms are jit-compiled for performance
- MoveIt! Compatible Constraints:
  - Start and goal configurations
  - Start and goal cartesian poses
- Generated trajectories respects the following constraints specified in the URDF:
  - Joint limits
  - Obstacle avoidance
  - Collision avoidance
  - Joint Velocity Limits
  - Joint Acceleration Limits
  - Joint Torque Limits
  
## Installation
~~~bash
pip install cfs-ros
~~~

## Usage

Please refer to the [examples](examples) for usage examples.

## Citation

If you find this work useful, please cite:

```bibtex
@article{liu2018convex,
  title={The convex feasible set algorithm for real time optimization in motion planning},
  author={Liu, Changliu and Lin, Chung-Yen and Tomizuka, Masayoshi},
  journal={SIAM Journal on Control and optimization},
  volume={56},
  number={4},
  pages={2712--2733},
  year={2018},
  publisher={SIAM}
}
@article{liu2017real,
  title={Real time trajectory optimization for nonlinear robotic systems: Relaxation and convexification},
  author={Liu, Changliu and Tomizuka, Masayoshi},
  journal={Systems \& Control Letters},
  volume={108},
  pages={56--63},
  year={2017},
  publisher={Elsevier}
}
```
