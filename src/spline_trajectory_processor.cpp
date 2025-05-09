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

#include <openmore/trajectories_processors/spline_trajectory_processor.h>

namespace openmore
{
void fromEigen2Vector(const Eigen::VectorXd& eigen, std::vector<double> vector)
{
  vector.clear();
  vector.resize(eigen.rows());
  Eigen::VectorXd::Map(&vector[0], eigen.rows()) = eigen;
}

bool SplineTrajectoryProcessor::interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling)
{
  // Spline interpolator uses the target scaling factor
  updated_scaling = target_scaling;

  if (trj_.empty())
  {
    CNR_ERROR(logger_, "Trajectory is empty or it has not been computed yet");
    return false;
  }

  if ((time - trj_.at(0)->time_from_start_) < 0)
  {
    *(pnt->state_) = *(trj_.front()->state_);  // copy the state
    pnt->time_from_start_ = trj_.front()->time_from_start_;
    return true;
  }

  if ((time - trj_.back()->time_from_start_) >= 0)
  {
    *(pnt->state_) = *(trj_.back()->state_);  // copy the state
    pnt->time_from_start_ = trj_.back()->time_from_start_;
    return true;
  }

  unsigned int nAx = trj_.at(0)->state_->pos_.size();

  for (unsigned int iPnt = 1; iPnt < trj_.size(); iPnt++)
  {
    if (((time - trj_.at(iPnt)->time_from_start_) < 0) && ((time - trj_.at(iPnt - 1)->time_from_start_) >= 0))
    {
      pnt->state_->pos_.resize(nAx, 0.0);
      pnt->state_->vel_.resize(nAx, 0.0);
      pnt->state_->acc_.resize(nAx, 0.0);
      pnt->state_->eff_.resize(nAx, 0.0);
      pnt->time_from_start_ = time;
      double delta_time = std::max(1.0e-6, (trj_.at(iPnt)->time_from_start_ - trj_.at(iPnt - 1)->time_from_start_));
      double t = (time - trj_.at(iPnt - 1)->time_from_start_);
      double ratio = t / delta_time;
      for (unsigned int iAx = 0; iAx < nAx; iAx++)
      {
        // spline
        if (spline_order_ == spline_order_t::ZERO)
        {
          pnt->state_->pos_.at(iAx) =
              trj_.at(iPnt - 1)->state_->pos_.at(iAx) + ratio * (trj_.at(iPnt)->state_->pos_.at(iAx) - trj_.at(iPnt - 1)->state_->pos_.at(iAx));
          pnt->state_->vel_.at(iAx) = (trj_.at(iPnt)->state_->pos_.at(iAx) - trj_.at(iPnt - 1)->state_->pos_.at(iAx)) / delta_time;
        }
        else if (spline_order_ == spline_order_t::ONE)
        {
          double& p0_1 = trj_.at(iPnt - 1)->state_->pos_.at(iAx);
          double& p0_2 = trj_.at(iPnt - 1)->state_->vel_.at(iAx);
          double& pf_1 = trj_.at(iPnt)->state_->pos_.at(iAx);
          double& pf_2 = trj_.at(iPnt)->state_->vel_.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = -1.0 / (delta_time * delta_time) * (p0_1 * 3.0 - pf_1 * 3.0 + delta_time * p0_2 * 2.0 + delta_time * pf_2);
          double c4 = 1.0 / (delta_time * delta_time * delta_time) * (p0_1 * 2.0 - pf_1 * 2.0 + delta_time * p0_2 + delta_time * pf_2);

          pnt->state_->pos_.at(iAx) = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t);
          pnt->state_->vel_.at(iAx) = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0;
          pnt->state_->acc_.at(iAx) = c3 * 2.0 + c4 * t * 6.0;
        }
        else if (spline_order_ == spline_order_t::TWO)
        {
          double& p0_1 = trj_.at(iPnt - 1)->state_->pos_.at(iAx);
          double& p0_2 = trj_.at(iPnt - 1)->state_->vel_.at(iAx);
          double& p0_3 = trj_.at(iPnt - 1)->state_->acc_.at(iAx);
          double& pf_1 = trj_.at(iPnt)->state_->pos_.at(iAx);
          double& pf_2 = trj_.at(iPnt)->state_->vel_.at(iAx);
          double& pf_3 = trj_.at(iPnt)->state_->acc_.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 1.0 / (delta_time * delta_time * delta_time) *
                      (p0_1 * 2.0E1 - pf_1 * 2.0E1 + delta_time * p0_2 * 1.2E1 + delta_time * pf_2 * 8.0 + (delta_time * delta_time) * p0_3 * 3.0 -
                       (delta_time * delta_time) * pf_3) *
                      (-1.0 / 2.0);
          double c5 = 1.0 / (delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 3.0E1 - pf_1 * 3.0E1 + delta_time * p0_2 * 1.6E1 + delta_time * pf_2 * 1.4E1 + (delta_time * delta_time) * p0_3 * 3.0 -
                       (delta_time * delta_time) * pf_3 * 2.0) *
                      (1.0 / 2.0);
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.2E1 - pf_1 * 1.2E1 + delta_time * p0_2 * 6.0 + delta_time * pf_2 * 6.0 + (delta_time * delta_time) * p0_3 -
                       (delta_time * delta_time) * pf_3) *
                      (-1.0 / 2.0);

          pnt->state_->pos_.at(iAx) = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t);
          pnt->state_->vel_.at(iAx) = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0;
          pnt->state_->acc_.at(iAx) = c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1;
        }
        else if (spline_order_ == spline_order_t::THREE)
        {
          double& p0_1 = trj_.at(iPnt - 1)->state_->pos_.at(iAx);
          double& p0_2 = trj_.at(iPnt - 1)->state_->vel_.at(iAx);
          double& p0_3 = trj_.at(iPnt - 1)->state_->acc_.at(iAx);
          double& pf_1 = trj_.at(iPnt)->state_->pos_.at(iAx);
          double& pf_2 = trj_.at(iPnt)->state_->vel_.at(iAx);
          double& pf_3 = trj_.at(iPnt)->state_->acc_.at(iAx);
          // initial and final jerks set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 0.0;
          double c5 = 1.0 / (delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.4E1 - pf_1 * 1.4E1 + delta_time * p0_2 * 8.0 + delta_time * pf_2 * 6.0 + (delta_time * delta_time) * p0_3 * 2.0 -
                       (delta_time * delta_time) * pf_3) *
                      (-5.0 / 2.0);
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 8.4E1 - pf_1 * 8.4E1 + delta_time * p0_2 * 4.5E1 + delta_time * pf_2 * 3.9E1 + (delta_time * delta_time) * p0_3 * 1.0E1 -
                       (delta_time * delta_time) * pf_3 * 7.0);
          double c7 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.4E2 - pf_1 * 1.4E2 + delta_time * p0_2 * 7.2E1 + delta_time * pf_2 * 6.8E1 + (delta_time * delta_time) * p0_3 * 1.5E1 -
                       (delta_time * delta_time) * pf_3 * 1.3E1) *
                      (-1.0 / 2.0);
          double c8 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.0E1 - pf_1 * 1.0E1 + delta_time * p0_2 * 5.0 + delta_time * pf_2 * 5.0 + (delta_time * delta_time) * p0_3 -
                       (delta_time * delta_time) * pf_3) *
                      2.0;

          pnt->state_->pos_.at(iAx) = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t) +
                                      c7 * (t * t * t * t * t * t) + c8 * (t * t * t * t * t * t * t);
          pnt->state_->vel_.at(iAx) = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0 +
                                      c7 * (t * t * t * t * t) * 6.0 + c8 * (t * t * t * t * t * t) * 7.0;
          pnt->state_->acc_.at(iAx) =
              c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1 + c7 * (t * t * t * t) * 3.0E1 + c8 * (t * t * t * t * t) * 4.2E1;
        }
        else if (spline_order_ == spline_order_t::FOUR)
        {
          double& p0_1 = trj_.at(iPnt - 1)->state_->pos_.at(iAx);
          double& p0_2 = trj_.at(iPnt - 1)->state_->vel_.at(iAx);
          double& p0_3 = trj_.at(iPnt - 1)->state_->acc_.at(iAx);
          double& pf_1 = trj_.at(iPnt)->state_->pos_.at(iAx);
          double& pf_2 = trj_.at(iPnt)->state_->vel_.at(iAx);
          double& pf_3 = trj_.at(iPnt)->state_->acc_.at(iAx);
          // initial and final jerks and spans set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3 * (1.0 / 2.0);
          double c4 = 0.0;
          double c5 = 0.0;
          double c6 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 3.6E1 - pf_1 * 3.6E1 + delta_time * p0_2 * 2.0E1 + delta_time * pf_2 * 1.6E1 + (delta_time * delta_time) * p0_3 * 5.0 -
                       (delta_time * delta_time) * pf_3 * 3.0) *
                      (-7.0 / 2.0);
          double c7 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.2E2 - pf_1 * 1.2E2 + delta_time * p0_2 * 6.4E1 + delta_time * pf_2 * 5.6E1 + (delta_time * delta_time) * p0_3 * 1.5E1 -
                       (delta_time * delta_time) * pf_3 * 1.1E1) *
                      (7.0 / 2.0);
          double c8 = -1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 5.4E2 - pf_1 * 5.4E2 + delta_time * p0_2 * 2.8E2 + delta_time * pf_2 * 2.6E2 + (delta_time * delta_time) * p0_3 * 6.3E1 -
                       (delta_time * delta_time) * pf_3 * 5.3E1);
          double c9 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                      (p0_1 * 1.26E2 - pf_1 * 1.26E2 + delta_time * p0_2 * 6.4E1 + delta_time * pf_2 * 6.2E1 + (delta_time * delta_time) * p0_3 * 1.4E1 -
                       (delta_time * delta_time) * pf_3 * 1.3E1) *
                      (5.0 / 2.0);
          double c10 = 1.0 / (delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time * delta_time) *
                       (p0_1 * 2.8E1 - pf_1 * 2.8E1 + delta_time * p0_2 * 1.4E1 + delta_time * pf_2 * 1.4E1 + (delta_time * delta_time) * p0_3 * 3.0 -
                        (delta_time * delta_time) * pf_3 * 3.0) *
                       (-5.0 / 2.0);

          pnt->state_->pos_.at(iAx) = c1 + c2 * t + c3 * (t * t) + c4 * (t * t * t) + c5 * (t * t * t * t) + c6 * (t * t * t * t * t) +
                                      c7 * (t * t * t * t * t * t) + c8 * (t * t * t * t * t * t * t) + c9 * (t * t * t * t * t * t * t * t) +
                                      c10 * (t * t * t * t * t * t * t * t * t);
          pnt->state_->vel_.at(iAx) = c2 + c3 * t * 2.0 + c4 * (t * t) * 3.0 + c5 * (t * t * t) * 4.0 + c6 * (t * t * t * t) * 5.0 +
                                      c7 * (t * t * t * t * t) * 6.0 + c8 * (t * t * t * t * t * t) * 7.0 + c9 * (t * t * t * t * t * t * t) * 8.0 +
                                      c10 * (t * t * t * t * t * t * t * t) * 9.0;
          pnt->state_->acc_.at(iAx) = c3 * 2.0 + c4 * t * 6.0 + c5 * (t * t) * 1.2E1 + c6 * (t * t * t) * 2.0E1 + c7 * (t * t * t * t) * 3.0E1 +
                                      c8 * (t * t * t * t * t) * 4.2E1 + c9 * (t * t * t * t * t * t) * 5.6E1 + c10 * (t * t * t * t * t * t * t) * 7.2E1;
        }
        pnt->state_->vel_.at(iAx) *= target_scaling;
        pnt->state_->acc_.at(iAx) *= target_scaling * target_scaling;
      }
      break;
    }
  }

  return true;
}

bool SplineTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
{
  TrajectoryProcessorBase::init(constraints, param_ns, logger);

  int spline_order;
  std::string what, full_param_name = param_ns + "/spline_order";
  if (cnr::param::has(full_param_name, what))
  {
    if (not cnr::param::get(full_param_name, spline_order, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n" << what);
      return false;
    }
    else
    {
      switch (spline_order)
      {
        case 0:
          spline_order_ = spline_order_t::ZERO;
          break;
        case 1:
          spline_order_ = spline_order_t::ONE;
          break;
        case 2:
          spline_order_ = spline_order_t::TWO;
          break;
        case 3:
          spline_order_ = spline_order_t::THREE;
          break;
        case 4:
          spline_order_ = spline_order_t::FOUR;
          break;
        default:
          CNR_ERROR(logger_, "Spline order should be 0, 1, 2, 3 or 4");
          return false;
      }
    }
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n" << what);
    return false;
  }

  return true;
}
bool SplineTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                                     const std::vector<Eigen::VectorXd>& path)
{
  TrajectoryProcessorBase::init(constraints, param_ns, logger, path);

  int spline_order;
  std::string what, full_param_name = param_ns + "/spline_order";
  if (cnr::param::has(full_param_name, what))
  {
    if (not cnr::param::get(full_param_name, spline_order, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n" << what);
      return false;
    }
    else
    {
      switch (spline_order)
      {
        case 0:
          spline_order_ = spline_order_t::ZERO;
          break;
        case 1:
          spline_order_ = spline_order_t::ONE;
          break;
        case 2:
          spline_order_ = spline_order_t::TWO;
          break;
        case 3:
          spline_order_ = spline_order_t::THREE;
          break;
        case 4:
          spline_order_ = spline_order_t::FOUR;
          break;
        default:
          CNR_ERROR(logger_, "Spline order should be 0, 1, 2, 3 or 4");
          return false;
      }
    }
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n" << what);
    return false;
  }

  return true;
}

}  // namespace openmore
