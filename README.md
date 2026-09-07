# snake_raven_controller

The original ROS control node and C++ kinematics for SnakeRaven, a 3D-printed snake-like continuum instrument for the RAVEN II surgical robot, together with the `raven_2` modifications that add a velocity joint control mode.

> **This package has been superseded.** It was merged with [`vision_servo_control_snakeraven`](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven) into **[SnakeRaven-Project](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project)**, which adds the endoscopic vision system, dual-arm teleoperation, IBVS-assisted control and autonomous waypoint navigation.
>
> **Go there instead.** Its README also carries the full `raven_2` modification map, the source map and the node rates that were originally documented here.
>
> This repository is kept as the original controller implementation, from when SnakeRaven was first built.

**Status:** archived, superseded by SnakeRaven-Project.

## The method

The kinematics and teleoperation implemented here are published in:

> A. Razjigaev, A. K. Pandey, D. Howard, J. Roberts and L. Wu, "SnakeRaven: Teleoperation of a 3D Printed Snake-like Manipulator Integrated to the RAVEN II Surgical Robot," *2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2021, pp. 5282–5288. [doi:10.1109/IROS51168.2021.9636878](https://doi.org/10.1109/IROS51168.2021.9636878)

Full derivation: [PhD thesis](https://eprints.qut.edu.au/235042/).

## What is here

- `talkerSnakeRaven` — the main ROS node. Subscribes to `/joint_states`, publishes joint deltas on `/raven_jointmove`.
- `listenerSnakeRaven` — a stand-in for the real RAVEN II node, for testing without the robot.
- `SnakeRaven.cpp` / `.h` — forward and inverse kinematics for the continuum section.
- `Raven_Controller.cpp` / `.h` — console interaction and mode handling, on a ROS thread and a console thread.
- `raven_2/` — modified RAVEN II files adding the velocity joint control mode.

The RAVEN II software drives the tool centre point. SnakeRaven's continuum section needs its joints commanded individually, so `raven_2` gains a velocity joint control mode that accepts incremental joint updates over a ROS topic. That is what the modified files are for.

Built against ROS Kinetic and RAVEN II release 18_05. Requires [Eigen](https://eigen.tuxfamily.org), header-only and not vendored: copy the `Eigen` subfolder into an `include/` folder in the package directory.

## Acknowledgements

The ROS integration started from [AutoCircle_generator](https://github.com/melodysu83/AutoCircle_generater), QUT's reference example for programming the RAVEN II over ROS. It demonstrates tool-centre-point control; this work extends the approach to joint-level control.

## Licence

MIT — see [LICENSE](LICENSE). The same licence as the [RAVEN II software](https://github.com/uw-biorobotics/raven2) this builds on.

<!-- CONFIRMED by Andrew 2026-09-07: this repo is MIT after all. The earlier
     "unlicensed" note came from generalising his statement about the other repos
     rather than checking this one — the second time that has happened on this
     subsystem. Stop inferring licence state; read each repo's sidebar. -->


## Questions

Written by Andrew Razjigaev. Questions: andrew_razjigaev@outlook.com
