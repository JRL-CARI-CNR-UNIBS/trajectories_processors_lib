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

#include <openmore/trajectories_processors/trajectory_processor_base.h>

/**
 * @file SplineTrajectoryProcessor.h
 * @brief Contains the declaration of the SplineTrajectoryProcessor class.
 */

namespace openmore
{
class SplineTrajectoryProcessor;
typedef std::shared_ptr<SplineTrajectoryProcessor> SplineTrajectoryProcessorPtr;

/**
 * @brief The SplineTrajectoryProcessor class processes trajectories using spline interpolation of different orders.
 */

class SplineTrajectoryProcessor: public TrajectoryProcessorBase
{
public:
  /**
   * @brief Enum for spline order.
   *        - ZERO: position continuous
   *        - ONE: velocity continuous
   *        - TWO: acceleration continuous
   *        - THREE: jerk continuous
   *        - FOUR: snap continuous
   */
  enum class spline_order_t
  {
    ZERO  = 0,
    ONE   = 1,
    TWO   = 2,
    THREE = 3,
    FOUR  = 4
  };

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
  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger):
    TrajectoryProcessorBase(constraints,param_ns,logger){}

  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path):
    TrajectoryProcessorBase(constraints,param_ns,logger,path){}

  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const spline_order_t& spline_order):
    TrajectoryProcessorBase(constraints,param_ns,logger),spline_order_(spline_order){}

  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const spline_order_t& spline_order):
    TrajectoryProcessorBase(constraints,param_ns,logger,path),spline_order_(spline_order){}

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @return
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger) override;
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path) override;

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
   * @param initial_state The initial robot state.
   * @param final_state The final robot state.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  //  virtual bool computeTrj() override;
  //  virtual bool computeTrj(const RobotState& initial_state) override;
  //  virtual bool computeTrj(const RobotState& initial_state, const RobotState& final_state) override;


  /**
   * @brief Pure virtual function to interpolate a trajectory point.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param target_scaling Target scaling factor used for the interpolation.
   * @param updated_scaling Actual scaling factor used for the interpolation. It should always be <= target_scaling.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling) override;
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling) override;

};
}
