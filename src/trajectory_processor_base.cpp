#include <trajectories_processors_lib/trajectory_processor_base.h>

namespace trajectories_processors
{
bool TrajectoryProcessorBase::init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
{
  trj_.clear();
  path_.clear();
  logger_ = logger;
  param_ns_ = std::move(param_ns);
  kinodynamic_constraints_ = std::move(constraints);
  return true;
}
bool TrajectoryProcessorBase::init(const KinodynamicConstraints& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path)
{
  trj_.clear();
  logger_ = logger;
  path_ = std::move(path);
  param_ns_ = std::move(param_ns);
  kinodynamic_constraints_ = std::move(constraints);
  return true;
}
}
