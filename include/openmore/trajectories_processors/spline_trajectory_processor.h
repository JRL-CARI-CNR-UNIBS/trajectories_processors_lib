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
   * @brief Enum representing spline interpolation orders.
   * - ZERO: Position continuity.
   * - ONE: Velocity continuity.
   * - TWO: Acceleration continuity.
   * - THREE: Jerk continuity.
   * - FOUR: Snap continuity.
   */
  enum class spline_order_t
  {
    ZERO = 0,  /**< Position continuity. */
    ONE = 1,   /**< Velocity continuity. */
    TWO = 2,   /**< Acceleration continuity. */
    THREE = 3, /**< Jerk continuity. */
    FOUR = 4   /**< Snap continuity. */
  };

protected:
  /**
   * @brief The spline interpolation order. Defaults to ONE (velocity continuity).
   */
  spline_order_t spline_order_ = spline_order_t::ONE;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   * Requires a call to init() aftwrwards.
   */
  SplineTrajectoryProcessor():
    TrajectoryProcessorBase(){}

  /**
   * @brief Constructor.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The namespace for parameter retrieval.
   * @param logger The logger instance.
   */
  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger):
    TrajectoryProcessorBase(constraints,param_ns,logger){}

  /**
   * @brief Constructor with a predefined path.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The namespace for parameter retrieval.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   */
  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path):
    TrajectoryProcessorBase(constraints,param_ns,logger,path){}

  /**
   * @brief Constructor with a predefined spline order.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The namespace for parameter retrieval.
   * @param logger The logger instance.
   * @param spline_order The spline interpolation order.
   */
  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const spline_order_t& spline_order):
    TrajectoryProcessorBase(constraints,param_ns,logger),spline_order_(spline_order){}

  /**
   * @brief Constructor with both a predefined path and spline order.
   * @param constraints The kinodynamic constraints.
   * @param param_ns The namespace for parameter retrieval.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   * @param spline_order The spline interpolation order.
   */
  SplineTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const spline_order_t& spline_order):
    TrajectoryProcessorBase(constraints,param_ns,logger,path),spline_order_(spline_order){}


  /**
   * @brief Initializes the SplineTrajectoryProcessor object with a predefined path.
   *
   * This function should be called when the default constructor is used. It is mainly used in plugin-based architectures.
   *
   * @param constraints The kinodynamic constraints for trajectory generation.
   * @param param_ns The namespace for parameter retrieval.
   * @param logger The logger instance.
   * @param path The predefined path for time-law computation.
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger) override;
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path) override;

  /**
   * @brief Sets the spline interpolation order and clears the existing trajectory.
   * @param spline_order The new spline interpolation order.
   */
  void setSplineOrder(const spline_order_t& spline_order)
  {
    spline_order_ = spline_order;
    trj_.clear();
  }

  /**
   * @brief Interpolates a trajectory point at a given time using the specified spline order.
   *
   * This function is overridden to provide spline-based interpolation of trajectory points.
   *
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param target_scaling The target scaling factor for interpolation.
   * @param updated_scaling The actual scaling factor used for interpolation (always <= target_scaling).
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling) override;
  using TrajectoryProcessorBase::interpolate; /**< Brings other overloads of the interpolate function into the scope. */
};
}
