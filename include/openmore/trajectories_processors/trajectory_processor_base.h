/*
Copyright (c) 2024

JRL-CARI CNR-STIIMA/UNIBS
Cesare Tonola, c.tonola001@unibs.it
Manuel Beschi, manuel.beschi@unibs.it

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <Eigen/Core>
#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>

/**
 * @file TrajectoryProcessorBase.h
 * @brief Contains the declaration of the TrajectoryProcessorBase class and related structures.
 */

namespace openmore
{
/**
 * @brief The RobotState struct represents a robot state, that is robot position, velocity, acceleration and effort.
 */
struct RobotState
{
public:
  std::vector<double> pos_; /**< Robot's position. */
  std::vector<double> vel_; /**< Robot's velocity. */
  std::vector<double> acc_; /**< Robot's acceleration. */
  std::vector<double> eff_; /**< Robot's effort. */
};
typedef std::shared_ptr<RobotState> RobotStatePtr;

/**
 * @brief Overloads the << operator for RobotState for printing.
 * @param os The output stream.
 * @param state The RobotState object.
 * @return The modified output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const RobotState& state)
{
  os << "pos: [ ";
  for (const auto& pos : state.pos_)
    os << pos << " ";
  os << "] vel: [ ";
  for (const auto& vel : state.vel_)
    os << vel << " ";
  os << "] acc: [ ";
  for (const auto& acc : state.acc_)
    os << acc << " ";
  os << "] eff: [ ";
  for (const auto& eff : state.eff_)
    os << eff << " ";
  os << "]";
  return os;
}
/**
 * @brief The TrjPoint struct represents a trajectory point consisting of a robot state and a timestamp from the start of the trajectory.
 */
struct TrjPoint
{
public:
  RobotStatePtr state_;    /**< The robot state at the trajectory point. */
  double time_from_start_; /**< Time from the start of the trajectory. */

  /**
   * @brief Default constructor.
   * Initializes the state and sets the time to infinity (default value).
   */
  TrjPoint()
  {
    state_ = std::make_shared<RobotState>();
    time_from_start_ = std::numeric_limits<double>::infinity();
  }
};
typedef std::shared_ptr<TrjPoint> TrjPointPtr;

/**
 * @brief Overloads the << operator for TrjPoint for printing.
 * @param os The output stream.
 * @param point The TrjPoint object.
 * @return The modified output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const TrjPoint& point)
{
  os << "time: " << point.time_from_start_ << " -> ";
  os << *point.state_;
  return os;
}

/**
 * @brief The kinodynamic_constraints struct represents the kinodynamic constraints of the robot,  including limits for position, velocity, acceleration, and
 * effort.
 */
struct KinodynamicConstraints
{
public:
  Eigen::VectorXd max_pos_; /**< Maximum position limits. */
  Eigen::VectorXd max_vel_; /**< Maximum velocity limits. */
  Eigen::VectorXd max_acc_; /**< Maximum acceleration limits. */
  Eigen::VectorXd max_eff_; /**< Maximum effort limits. */

  Eigen::VectorXd min_pos_; /**< Minimum position limits. */
  Eigen::VectorXd min_vel_; /**< Minimum velocity limits. */
  Eigen::VectorXd min_acc_; /**< Minimum acceleration limits. */
  Eigen::VectorXd min_eff_; /**< Minimum effort limits. */
};
typedef std::shared_ptr<KinodynamicConstraints> KinodynamicConstraintsPtr;

/**
 * @brief Overloads the << operator for KinodynamicConstraints for printing.
 * @param os The output stream.
 * @param constraints The KinodynamicConstraints object.
 * @return The modified output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const KinodynamicConstraints& constraints)
{
  // Position limits
  if (constraints.min_pos_.cols() != 0)
    os << "min pos: [" << constraints.min_pos_.transpose();
  else
    os << "min pos: [n.d.";

  if (constraints.max_pos_.cols() != 0)
    os << "] max pos: [" << constraints.max_pos_.transpose();
  else
    os << "] max pos: [n.d.";

  // Velocity limits
  if (constraints.min_vel_.cols() != 0)
    os << "]\nmin vel: [" << constraints.min_vel_.transpose();
  else
    os << "]\nmin vel: [n.d.";

  if (constraints.max_vel_.cols() != 0)
    os << "] max vel: [" << constraints.max_vel_.transpose();
  else
    os << "] max vel: [n.d.";

  // Acceleration limits
  if (constraints.min_acc_.cols() != 0)
    os << "]\nmin acc: [" << constraints.min_acc_.transpose();
  else
    os << "]\nmin acc: [n.d.";

  if (constraints.max_acc_.cols() != 0)
    os << "] max acc: [" << constraints.max_acc_.transpose();
  else
    os << "] max acc: [n.d.";

  // Effort limits
  if (constraints.min_eff_.cols() != 0)
    os << "]\nmin eff: [" << constraints.min_eff_.transpose();
  else
    os << "]\nmin eff: [n.d.";

  if (constraints.max_eff_.cols() != 0)
    os << "] max eff: [" << constraints.max_eff_.transpose();
  else
    os << "] max eff: [n.d.";

  os << "]";

  return os;
}

class TrajectoryProcessorBase;
typedef std::shared_ptr<TrajectoryProcessorBase> TrajectoryProcessorBasePtr;

/**
 * @brief The TrajectoryProcessorBase class provides a base class for processing trajectories.
 */
class TrajectoryProcessorBase : public std::enable_shared_from_this<TrajectoryProcessorBase>
{
protected:
  /**
   * @brief Robot's kinodynamic constraints.
   */
  KinodynamicConstraintsPtr kinodynamic_constraints_;

  /**
   * @brief The path for which the time law is computed.
   */
  std::vector<Eigen::VectorXd> path_;

  /**
   * @brief The computed trajectory.
   */
  std::deque<TrjPointPtr> trj_;

  /**
   * @brief Namespace for parameter retrieval using cnr_param.
   */
  std::string param_ns_;

  /**
   * @brief Logger for debugging and tracing.
   */
  cnr_logger::TraceLoggerPtr logger_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   * Requires a call to init() aftwrwards.
   */
  TrajectoryProcessorBase()
  {
  }

  /**
   * @brief Parameterized constructor.
   * @param constraints Kinodynamic constraints.
   * @param param_ns Parameter namespace.
   * @param logger Logger instance.
   */
  TrajectoryProcessorBase(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
    : kinodynamic_constraints_(constraints), param_ns_(param_ns), logger_(logger)
  {
  }

  /**
   * @brief Constructor with path initialization.
   * @param constraints Kinodynamic constraints.
   * @param param_ns Parameter namespace.
   * @param logger Logger instance.
   * @param path Path for time-law computation.
   */
  TrajectoryProcessorBase(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                          const std::vector<Eigen::VectorXd>& path)
    : kinodynamic_constraints_(constraints), path_(path), param_ns_(param_ns), logger_(logger)
  {
  }

  /**
   * @brief Initializes the TrajectoryProcessor object with a predefined path.
   *
   * This function should be called when the default constructor is used. It initializes the object with a specific path and constraints.
   *
   * @param constraints The kinodynamic constraints of the robot to be used for trajectory generation.
   * @param param_ns The namespace from which to read parameters using cnr_param.
   * @param logger The logger instance for debugging and tracing.
   * @param path The predefined path for time-law computation.
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                    const std::vector<Eigen::VectorXd>& path);

  /**
   * @brief Returns a shared pointer to the current instance.
   * @return A shared pointer to this TrajectoryProcessorBase instance.
   */
  TrajectoryProcessorBasePtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief Sets the path and clears the existing trajectory.
   *
   * This function updates the path used for trajectory computation and clears any previously computed trajectory.
   *
   * @param path The new path for which the time-law needs to be computed.
   */
  void setPath(const std::vector<Eigen::VectorXd>& path)
  {
    path_ = path;
    trj_.clear();
  }

  /**
   * @brief Sets the kinodynamic constraints of the robot and clears the existing trajectory.
   *
   * @param constraints The new kinodynamic constraints for trajectory processing.
   */
  void setConstraints(const KinodynamicConstraintsPtr& constraints)
  {
    kinodynamic_constraints_ = constraints;
    trj_.clear();
  }

  /**
   * @brief Gets the current path.
   * @return The current path as a vector of Eigen::VectorXd.
   */
  const std::vector<Eigen::VectorXd>& getPath() const
  {
    return path_;
  }

  /**
   * @brief Gets the computed trajectory.
   * @return The computed trajectory as a deque of trajectory points (TrjPointPtr).
   */
  const std::deque<TrjPointPtr>& getTrj() const
  {
    return trj_;
  }

  /**
   * @brief Returns the duration of the computed trajectory.
   *
   * @return The duration of the trajectory.
   * @throws std::runtime_error if the trajectory is empty.
   */
  const double& getTrjDuration() const
  {
    if (trj_.empty())
      throw std::runtime_error("trj is empty");

    return trj_.back()->time_from_start_;
  }

  /**
   * @brief Converts the computed trajectory to a YAML node.
   *
   * This function generates a YAML::Node representing the trajectory, including positions, velocities, accelerations, efforts, and timestamps.
   *
   * @return A YAML::Node containing the trajectory data.
   * @throws std::runtime_error if mandatory data (e.g., position or time) is missing.
   */
  YAML::Node toYAML() const;

  /**
   * @brief Pure virtual function to compute the trajectory.
   *
   * @param initial_state The initial state of the robot.
   * @param final_state The final state of the robot.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state) = 0;
  virtual bool computeTrj(const RobotStatePtr& initial_state);
  virtual bool computeTrj();

  /**
   * @brief Pure virtual function to interpolate a trajectory point at a given time.
   *
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param target_scaling The target scaling factor for interpolation.
   * @param updated_scaling The actual scaling factor used for interpolation (always <= target_scaling).
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling) = 0;
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling = 1.0);
};

namespace utils
{
/**
 * @brief Creates a trajectory from a YAML node.
 *
 * @param yaml The YAML node containing trajectory data.
 * @return A deque of trajectory points (TrjPointPtr).
 * @throws std::runtime_error if mandatory fields (e.g., positions or times) are missing or inconsistent.
 */
std::deque<TrjPointPtr> trjFromYAML(const YAML::Node& yaml);
}  // namespace utils

/**
 * @brief Overloads the << operator for printing a deque of trajectory points.
 * @param os The output stream.
 * @param trj The deque of trajectory points.
 * @return The modified output stream.
 */
inline std::ostream& operator<<(std::ostream& os, const std::deque<TrjPointPtr>& trj)
{
  for (const TrjPointPtr& pt : trj)
    os << *pt << "\n";
  return os;
}

}  // namespace openmore
