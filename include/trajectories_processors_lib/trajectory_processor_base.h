#pragma once

#include <cnr_logger/cnr_logger.h>
#include <graph_core/graph/path.h>

/**
 * @file TrajectoryProcessorBase.h
 * @brief Contains the declaration of the TrajectoryProcessorBase class and related structures.
 */

namespace trajectories_processors
{
using namespace graph::core;

/**
 * @brief The robot_state struct represents a robot state, that is robot position, velocity, acceleration and effort.
 */
struct robot_state
{
public:
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> acc_;
  std::vector<double> eff_;
};

/**
 * @brief The TrjPoint struct represents a trajectory point, that is made up of a robot state and a time from trajectory start.
 */
struct TrjPoint
{
public:
  robot_state state_;
  double time_from_start_;
};

/**
 * @brief The kinodynamic_constraints struct represents the kinodynamic constraints of the robot
 */
struct KinodynamicConstraints
{
public:
  Eigen::VectorXd max_vel_;
  Eigen::VectorXd max_acc_;
};

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
  KinodynamicConstraints kinodynamic_constraints_;

  /**
   * @brief The path for which the time law is computed.
   */
  std::vector<Eigen::VectorXd> path_;

  /**
   * @brief The logger for logging purposes.
   */
  std::deque<TrjPoint> trj_;

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
  TrajectoryProcessorBase(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger):
    kinodynamic_constraints_(constraints), param_ns_(param_ns), logger_(logger){}
  TrajectoryProcessorBase(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path):
    kinodynamic_constraints_(constraints), path_(path), param_ns_(param_ns), logger_(logger){}


  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @return
   */
  virtual bool init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger);
  virtual bool init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path);

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
  void setConstraints(const KinodynamicConstraints constraints)
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
  const std::deque<TrjPoint>& getTrj() const
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

    return trj_.back().time_from_start_;
  }
  /**
   * @brief Pure virtual function to compute the trajectory.
   * @param initial_state The initial trj point.
   * @param final_pnt The final trj point.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  virtual bool computeTrj() = 0;
  virtual bool computeTrj(const TrjPoint& initial_pnt) = 0;
  virtual bool computeTrj(const TrjPoint& initial_pnt, const TrjPoint& final_pnt) = 0;


  /**
   * @brief Pure virtual function to interpolate a trajectory point.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param scaling Scaling factor for interpolation.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPoint& pnt, const double& scaling = 1.0) = 0;

  /**
   * @brief Creates a clone of the TrajectoryProcessor object.
   * @return A shared pointer to the cloned TrajectoryProcessor object.
   */
  virtual TrajectoryProcessorBasePtr clone() = 0;
};
}
