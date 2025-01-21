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

#include <openmore/trajectories_processors/trajectory_processor_base.h>

namespace openmore
{
bool TrajectoryProcessorBase::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
{
  trj_.clear();
  path_.clear();
  logger_ = logger;
  param_ns_ = param_ns;
  kinodynamic_constraints_ = constraints;
  return true;
}
bool TrajectoryProcessorBase::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path)
{
  trj_.clear();
  logger_ = logger;
  path_ = path;
  param_ns_ = param_ns;
  kinodynamic_constraints_ = constraints;
  return true;
}

bool TrajectoryProcessorBase::computeTrj(const RobotStatePtr& initial_state)
{
  RobotStatePtr final_state;
  return computeTrj(initial_state,final_state);
}

bool TrajectoryProcessorBase::computeTrj()
{
  RobotStatePtr initial_state, final_state;
  return computeTrj(initial_state,final_state);
}

bool TrajectoryProcessorBase::interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling)
{
  double updated_scaling;
  return interpolate(time, pnt, target_scaling, updated_scaling);
}

YAML::Node TrajectoryProcessorBase::toYAML() const
{
  YAML::Node yaml;

  if (trj_.empty())
  {
    CNR_WARN(logger_, "Trajectory is empty. Returning an empty YAML node.");
    return yaml;
  }

  // Initialize YAML sequences for trajectory components
  YAML::Node positions, times, velocities, accelerations, efforts;

  // Set the style to flow style for a more compact representation
  positions.SetStyle(YAML::EmitterStyle::Block);
  velocities.SetStyle(YAML::EmitterStyle::Block);
  accelerations.SetStyle(YAML::EmitterStyle::Block);
  efforts.SetStyle(YAML::EmitterStyle::Block);

  bool has_velocities = false;
  bool has_accelerations = false;
  bool has_efforts = false;

  for (size_t i = 0; i < trj_.size(); ++i)
  {
    const auto& point = trj_[i];

    // Add mandatory fields
    if (point->state_->pos_.empty())
    {
      CNR_ERROR(logger_, "Position data missing for trajectory point " + std::to_string(i));
      throw std::runtime_error("Position data is required but missing.");
    }

    if (std::isinf(point->time_from_start_))
    {
      CNR_ERROR(logger_, "time_from_start is undefined for trajectory point " + std::to_string(i));
      throw std::runtime_error("time_from_start must be defined for all trajectory points.");
    }

    YAML::Node position_node(point->state_->pos_);
    position_node.SetStyle(YAML::EmitterStyle::Flow);
    positions.push_back(position_node);

    times.push_back(point->time_from_start_);

    // Handle optional fields
    if (!point->state_->vel_.empty())
    {
      YAML::Node velocity_node(point->state_->vel_);
      velocity_node.SetStyle(YAML::EmitterStyle::Flow);
      velocities.push_back(velocity_node);
      has_velocities = true;
    }
    else
    {
      if (has_velocities)
      {
        CNR_WARN(logger_, "Velocity data missing for trajectory point " + std::to_string(i) + ".");
      }
      velocities.push_back(YAML::Node()); // Add empty entry
    }

    if (!point->state_->acc_.empty())
    {
      YAML::Node acceleration_node(point->state_->acc_);
      acceleration_node.SetStyle(YAML::EmitterStyle::Flow);
      accelerations.push_back(acceleration_node);
      has_accelerations = true;
    }
    else
    {
      if (has_accelerations)
      {
        CNR_WARN(logger_, "Acceleration data missing for trajectory point " + std::to_string(i) + ".");
      }
      accelerations.push_back(YAML::Node()); // Add empty entry
    }

    if (!point->state_->eff_.empty())
    {
      YAML::Node effort_node(point->state_->eff_);
      effort_node.SetStyle(YAML::EmitterStyle::Flow);
      efforts.push_back(effort_node);
      has_efforts = true;
    }
    else
    {
      if (has_efforts)
      {
        CNR_WARN(logger_, "Effort data missing for trajectory point " + std::to_string(i) + ".");
      }
      efforts.push_back(YAML::Node()); // Add empty entry
    }
  }

  // Assign mandatory fields to the YAML structure
  yaml["times"] = times;
  yaml["positions"] = positions;

  // Assign optional fields only if they contain at least one non-empty value
  if (has_velocities)
  {
    yaml["velocities"] = velocities;
  }
  if (has_accelerations)
  {
    yaml["accelerations"] = accelerations;
  }
  if (has_efforts)
  {
    yaml["efforts"] = efforts;
  }

  return yaml;
}

namespace utils
{
std::deque<TrjPointPtr> trjFromYAML(const YAML::Node& yaml)
{

  // Ensure mandatory fields are present
  if (!yaml["positions"] || !yaml["times"])
  {
    throw std::runtime_error("YAML must contain 'positions' and 'times' fields.");
  }

  const auto& positions = yaml["positions"];
  const auto& times = yaml["times"];

  if (positions.size() != times.size())
  {
    throw std::runtime_error("Inconsistent sequence lengths between 'positions' and 'times'.");
  }

  std::deque<TrjPointPtr> trajectory;

  // Optional fields
  const auto& velocities = yaml["velocities"];
  const auto& accelerations = yaml["accelerations"];
  const auto& efforts = yaml["efforts"];

  // Check optional field sizes if they exist
  if (velocities && velocities.size() != positions.size())
  {
    throw std::runtime_error("Inconsistent sequence lengths between 'positions' and 'velocities'.");
  }
  if (accelerations && accelerations.size() != positions.size())
  {
    throw std::runtime_error("Inconsistent sequence lengths between 'positions' and 'accelerations'.");
  }
  if (efforts && efforts.size() != positions.size())
  {
    throw std::runtime_error("Inconsistent sequence lengths between 'positions' and 'efforts'.");
  }

  // Populate trajectory points
  for (size_t i = 0; i < positions.size(); ++i)
  {
    auto point = std::make_shared<TrjPoint>();

    // Mandatory fields
    if (!positions[i])
    {
      throw std::runtime_error("Position data is undefined for trajectory point " + std::to_string(i) + ".");
    }
    point->state_->pos_ = positions[i].as<std::vector<double>>();

    if (!times[i] || std::isnan(times[i].as<double>()))
    {
      throw std::runtime_error("time_from_start is undefined for trajectory point " + std::to_string(i) + ".");
    }
    point->time_from_start_ = times[i].as<double>();

    // Optional fields
    if (velocities && velocities[i])
    {
      point->state_->vel_ = velocities[i].as<std::vector<double>>();
    }

    if (accelerations && accelerations[i])
    {
      point->state_->acc_ = accelerations[i].as<std::vector<double>>();
    }

    if (efforts && efforts[i])
    {
      point->state_->eff_ = efforts[i].as<std::vector<double>>();
    }

    trajectory.push_back(point);
  }

  return trajectory;
}
}

}
