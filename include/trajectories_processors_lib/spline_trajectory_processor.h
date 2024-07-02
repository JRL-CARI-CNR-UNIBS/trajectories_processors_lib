#pragma once

#include <trajectories_processors_lib/trajectory_processor_base.h>

/**
 * @file SplineTrajectoryProcessor.h
 * @brief Contains the declaration of the SplineTrajectoryProcessor class.
 */

namespace trajectories_processors
{
class SplineTrajectoryProcessor;
typedef std::shared_ptr<SplineTrajectoryProcessor> SplineTrajectoryProcessorPtr;

/**
 * @brief The SplineTrajectoryProcessor class processes trajectories using spline interpolation of different orders.
 */

class SplineTrajectoryProcessor: public TrajectoryProcessorBase
{
  /**
   * @brief Enum for spline order.
   *        - ZERO: position continuous
   *        - ONE: velocity continuous
   *        - TWO: acceleration continuous
   *        - THREE: jerk continuous
   *        - FOUR: snap continuous
   */
  enum class spline_order_t {ZERO, ONE, TWO, THREE, FOUR};

protected:

  /**
   * @brief The order of the spline. Default order is ONE.
   */
  spline_order_t spline_order_ = spline_order_t::ONE;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructors.
   */
  SplineSplineTrajectoryProcessor(const KinodynamicConstraints& constraints, const cnr_logger::TraceLoggerPtr& logger):TrajectoryProcessorBase(constraints,logger){}
  SplineSplineTrajectoryProcessor(const KinodynamicConstraints& constraints, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path):
    TrajectoryProcessorBase(constraints,logger,path){}
  SplineSplineTrajectoryProcessor(const KinodynamicConstraints& constraints, const cnr_logger::TraceLoggerPtr& logger, const spline_order_t& spline_order):TrajectoryProcessorBase(constraints,logger),spline_order_(spline_order){}
  SplineSplineTrajectoryProcessor(const KinodynamicConstraints& constraints, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,const spline_order_t& spline_order):TrajectoryProcessorBase(constraints,logger,path),spline_order_(spline_order){}

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @return
   */
  virtual bool init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger) override;
  virtual bool init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path) override;

  /**
   * @brief Sets the order of the spline and clears the trajectory.
   * @param spline_order The order of the spline to set.
   */
  void setSplineOrder(const spline_order_t& spline_order)
  {
    spline_order_ = spline_order;
    trj_.clear();
  }

  /**
   * @brief Function to compute the trajectory.
   * @param initial_state The initial trj point.
   * @param final_pnt The final trj point.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  //  virtual bool computeTrj() override;
  //  virtual bool computeTrj(const TrjPoint& initial_pnt) override;
  //  virtual bool computeTrj(const TrjPoint& initial_pnt, const TrjPoint& final_pnt) override;


  /**
   * @brief Interpolates a trajectory point at a given time.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param scaling Scaling factor for interpolation.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPoint& pnt, const double& scaling = 1.0) override;
};
}
