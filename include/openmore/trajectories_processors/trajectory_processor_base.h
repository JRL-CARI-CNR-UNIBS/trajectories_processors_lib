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
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> acc_;
  std::vector<double> eff_;
};
typedef std::shared_ptr<RobotState> RobotStatePtr;

// Overload the << operator for RobotState
inline std::ostream& operator<<(std::ostream& os, const RobotState& state)
{
  os << "pos: [ ";
  for (const auto& pos : state.pos_) os << pos << " ";
  os << "] vel: [ ";
  for (const auto& vel : state.vel_) os << vel << " ";
  os << "] acc: [ ";
  for (const auto& acc : state.acc_) os << acc << " ";
  os << "] eff: [ ";
  for (const auto& eff : state.eff_) os << eff << " ";
  os << "]";
  return os;
}
/**
 * @brief The TrjPoint struct represents a trajectory point, that is made up of a robot state and a time from trajectory start.
 */
struct TrjPoint
{
public:
  RobotStatePtr state_;
  double time_from_start_;

  TrjPoint()
  {
    state_ = std::make_shared<RobotState>();
    time_from_start_ = std::numeric_limits<double>::infinity();
  }
};
typedef std::shared_ptr<TrjPoint> TrjPointPtr;

// Overload the << operator for TrjPoint
inline std::ostream& operator<<(std::ostream& os, const TrjPoint& point)
{
  os << "time: " << point.time_from_start_ << " -> ";
  os << *point.state_;
  return os;
}

/**
 * @brief The kinodynamic_constraints struct represents the kinodynamic constraints of the robot
 */
struct KinodynamicConstraints
{
public:
  Eigen::VectorXd max_pos_;
  Eigen::VectorXd max_vel_;
  Eigen::VectorXd max_acc_;
  Eigen::VectorXd max_eff_;

  Eigen::VectorXd min_pos_;
  Eigen::VectorXd min_vel_;
  Eigen::VectorXd min_acc_;
  Eigen::VectorXd min_eff_;
};
typedef std::shared_ptr<KinodynamicConstraints> KinodynamicConstraintsPtr;

// Overload the << operator for KinodynamicConstraints
inline std::ostream& operator<<(std::ostream& os, const KinodynamicConstraints& constraints)
{
  // Position limits
  if(constraints.min_pos_.cols()!=0)
    os << "min pos: [" << constraints.min_pos_.transpose();
  else
    os << "min pos: [n.d.";

  if(constraints.max_pos_.cols()!=0)
    os << "] max pos: [" << constraints.max_pos_.transpose();
  else
    os << "] max pos: [n.d.";

  // Velocity limits
  if(constraints.min_vel_.cols()!=0)
    os << "]\nmin vel: [" << constraints.min_vel_.transpose();
  else
    os << "]\nmin vel: [n.d.";

  if(constraints.max_vel_.cols()!=0)
    os << "] max vel: [" << constraints.max_vel_.transpose();
  else
    os << "] max vel: [n.d.";

  // Acceleration limits
  if(constraints.min_acc_.cols()!=0)
    os << "]\nmin acc: [" << constraints.min_acc_.transpose();
  else
    os << "]\nmin acc: [n.d.";

  if(constraints.max_acc_.cols()!=0)
    os << "] max acc: [" << constraints.max_acc_.transpose();
  else
    os << "] max acc: [n.d.";

  // Effort limits
  if(constraints.min_eff_.cols()!=0)
    os << "]\nmin eff: [" << constraints.min_eff_.transpose();
  else
    os << "]\nmin eff: [n.d.";

  if(constraints.max_eff_.cols()!=0)
    os << "] max eff: [" << constraints.max_eff_.transpose();
  else
    os << "] max eff: [n.d.";

  os << "]";

  return os;
}

class TrajectoryProcessorBase;
typedef std::shared_ptr<TrajectoryProcessorBase> TrajectoryProcessorBasePtr;

/**
 * @brief The TrajectoryProcessorBase class provides a base for processing trajectories.
 */
class TrajectoryProcessorBase: public std::enable_shared_from_this<TrajectoryProcessorBase>
{
protected:
  /**
   * @brief The kinodynamics constraints of the robot to be considered for trajectory generation.
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
   * @brief param_ns_ The namespace under which to read the parameters with cnr_param.
   */
  std::string param_ns_;

  /**
   * @brief logger_ the logger for logging purposes.
   */
  cnr_logger::TraceLoggerPtr logger_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructors.
   */
  TrajectoryProcessorBase(){} //need init() function call afterwards
  TrajectoryProcessorBase(const KinodynamicConstraintsPtr& constraints,
                          const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger):
    kinodynamic_constraints_(constraints), param_ns_(param_ns), logger_(logger){}

  TrajectoryProcessorBase(const KinodynamicConstraintsPtr& constraints,
                          const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger,
                          const std::vector<Eigen::VectorXd>& path):
    kinodynamic_constraints_(constraints), path_(path), param_ns_(param_ns), logger_(logger){}

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @return
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path);

  TrajectoryProcessorBasePtr pointer()
  {
    return shared_from_this();
  }

  /**
   * @brief Sets the path and clears the trajectory.
   * @param path The path for which the time-law need to be computed.
   */
  void setPath(const std::vector<Eigen::VectorXd>& path)
  {
    path_ = std::move(path);
    trj_.clear();
  }

  /**
   * @brief Set the kinodynamic constraints of the robot for trajectory processing and clear the trajectory.
   * @param constraints the kinodynamic constraints
   */
  void setConstraints(const KinodynamicConstraintsPtr& constraints)
  {
    kinodynamic_constraints_ = constraints;
    trj_.clear();
  }

  /**
   * @brief Gets the path.
   * @return The path.
   */
  const std::vector<Eigen::VectorXd>& getPath() const
  {
    return path_;
  }

  /**
   * @brief Gets the computed trajectory.
   * @return The computed trajectory.
   */
  const std::deque<TrjPointPtr>& getTrj() const
  {
    return trj_;
  }

  /**
   * @brief Return the duration of the trajectory
   * @return The trajectory duration.
   */
  const double& getTrjDuration() const
  {
    if(trj_.empty())
      throw std::runtime_error("trj is empty");

    return trj_.back()->time_from_start_;
  }
  /**
   * @brief Pure virtual function to compute the trajectory.
   * @param initial_state The initial robot state.
   * @param final_state The final robot state.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrj() = 0;
  virtual bool computeTrj(const RobotStatePtr& initial_state) = 0;
  virtual bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state) = 0;


  /**
   * @brief Pure virtual function to interpolate a trajectory point.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param scaling Scaling factor for interpolation.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& scaling = 1.0) = 0;
};

// Overload the << operator for std::deque<TrjPointPtr>
inline std::ostream& operator<<(std::ostream& os, const std::deque<TrjPointPtr>& trj)
{
  for(const TrjPointPtr& pt:trj)
    os<<*pt<<"\n";
  return os;
}

}
