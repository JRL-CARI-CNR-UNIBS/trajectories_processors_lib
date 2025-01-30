# trajectories_processors_lib
[![build](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/actions/workflows/build_and_install.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/actions/workflows/build_and_install.yaml)
[![clang-format](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/actions/workflows/clang-format.yaml)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/7fad0e98e668413e9207196bd38af72e)](https://app.codacy.com/gh/JRL-CARI-CNR-UNIBS/trajectories_processors_lib/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)
![Status](https://img.shields.io/badge/License-BSD3-green)

`trajectories_processors_lib` is a C++ library designed for processing and managing robot trajectories. 

## Classes and Structures

### `TrajectoryProcessorBase`

The `TrajectoryProcessorBase` class provides a base for processing trajectories. It includes the following members:

- **Members**:
  - `KinodynamicConstraints kinodynamic_constraints_`: The kinodynamic constraints of the robot.
  - `std::vector<Eigen::VectorXd> path_`: The path for which the time law need to be computed.
  - `std::deque<TrjPoint> trj_`: The computed trajectory.
  - `std::string param_ns_`: The namespace under which to read the parameters.
  - `cnr_logger::TraceLoggerPtr logger_`: The logger for logging purposes.

The main functions of the class are:
  - `computeTrj`: computes the time law for the given path, optionally with given starting and ending conditions.
  - `interpolate`: interpolates the computed trajectory to obtain the trajectory point at the desired time instant.

### `robot_state`

Represents a robot state, including position, velocity, acceleration, and effort.

- **Members**:
  - `std::vector<double> pos_`
  - `std::vector<double> vel_`
  - `std::vector<double> acc_`
  - `std::vector<double> eff_`

### `TrjPoint`

Represents a trajectory point, consisting of a robot state and a time from the trajectory start.

- **Members**:
  - `robot_state state_`
  - `double time_from_start_`

### `KinodynamicConstraints`

Represents the kinodynamic constraints of the robot.

- **Members**:
  - `Eigen::VectorXd min_pos_`
  - `Eigen::VectorXd min_vel_`
  - `Eigen::VectorXd min_acc_`
  - `Eigen::VectorXd min_eff_`
  - `Eigen::VectorXd max_pos_`
  - `Eigen::VectorXd max_vel_`
  - `Eigen::VectorXd max_acc_`
  - `Eigen::VectorXd max_eff_`

## Usage

Include the library headers in your project:
```cpp
#include <trajectories_processors_lib/trajectory_processor_base.h>
or
#include <trajectories_processors_lib/spline_trajectory_processor.h>
```

```trajectories_processors_lib/spline_trajectory_processor.h``` extends the base class by providing spline interpolations with selectable order from 0 to 4.

## Resources
You might be interested in [`moveit_trajectory_processor`](https://github.com/JRL-CARI-CNR-UNIBS/moveit_trajectory_processor).
