# trajectories_processors_lib

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
  - `Eigen::VectorXd max_vel_`
  - `Eigen::VectorXd max_acc_`
  - `Eigen::VectorXd min_vel_`
  - `Eigen::VectorXd min_acc_`

## Usage

Include the library headers in your project:
```cpp
#include <trajectories_processors_lib/trajectory_processor_base.h>
or
#include <trajectories_processors_lib/spline_trajectory_processor.h>
```

```trajectories_processors_lib/spline_trajectory_processor.h``` extends the base class by providing spline interpolations with selectable order from 0 to 4.
