#include <trajectories_processors_lib/spline_trajectory_processor.h>

namespace trajectories_processors
{

void fromEigen2Vector(const Eigen::VectorXd& eigen, std::vector<double> vector)
{
  vector.clear();
  vector.resize(eigen.rows());
  Eigen::VectorXd::Map(&vector[0], eigen.rows()) = eigen;
}

//bool SplineTrajectoryProcessor::computeTrj()
//{
//  robot_state initial_state;
//  fromEigen2Vector(path_.front(),initial_state.pos_);

//  TrjPoint initial_pnt;
//  initial_pnt.state_ = std::move(initial_state);
//  initial_pnt.time_from_start_ = 0.0;

//  return computeTrj(initial_pnt);
//}

//bool SplineTrajectoryProcessor::computeTrj(const TrjPoint& initial_pnt)
//{
//  robot_state final_state;
//  fromEigen2Vector(path_.back(),final_state.pos_);

//  TrjPoint final_pnt;
//  final_pnt.state_ = std::move(final_state);
//  final_pnt.time_from_start_ = 0.0;

//  return computeTrj(initial_pnt,final_pnt);
//}

//bool SplineTrajectoryProcessor::computeTrj(const TrjPoint& initial_pnt,const TrjPoint& final_pnt)
//{
//  if(path_.empty() or path_.size() == 1)
//  {
//    CNR_ERROR(logger_,"Path is empty or it is a single point");
//    return false;
//  }

//  size_t nAx = path_.front().rows();

//  trj_.clear();
//  trj_.push_back(initial_pnt);

//  if(spline_order_ == spline_order_t::ZERO)
//  {
//    if(not initial_pnt.state_.vel_.empty())
//      CNR_WARN(logger_,"Computing the trajectory with spline order zero will discard initial velocities");

//    if(kinodynamic_constraints_.max_vel_.rows() != nAx)
//    {
//      CNR_ERROR(logger_,"To compute a trajectory with spline order zero the velocities constraints must be defined");
//      return false;
//    }

//    double joint_time, slowest_joint_time;
//    for(size_t iPnt=1; iPnt<path_.size(); iPnt++)
//    {
//      slowest_joint_time = 0.0;
//      for(size_t iAx=0; iAx<nAx; iAx++)
//      {
//        joint_time = std::abs((path_.at(iPnt)[iAx]-path_.at(iPnt-1)[iAx])/kinodynamic_constraints_.max_vel_[iAx]);
//        if(joint_time>slowest_joint_time)
//          slowest_joint_time = std::move(joint_time);
//      }

//      TrjPoint pnt;
//      pnt.state_.pos_.resize(nAx);
//      pnt.state_.vel_.resize(nAx);

//      for(size_t iAx=0; iAx<nAx; iAx++)
//      {
//        pnt.state_.pos_[iAx] = path_.at(iPnt)[iAx];
//        pnt.state_.vel_[iAx] = (path_.at(iPnt)[iAx]-path_.at(iPnt-1)[iAx])/slowest_joint_time;
//      }
//      pnt.time_from_start_ = trj_.back().time_from_start_ + slowest_joint_time;
//      trj_.push_back(pnt);
//    }
//  }

//  if(spline_order_ == spline_order_t::ONE)
//  {
//    if(kinodynamic_constraints_.max_vel_.rows() != nAx)
//    {
//      CNR_ERROR(logger_,"To compute a trajectory with spline_order_ == spline_order_t::ZERO, the velocities constraint must be defined");
//      return false;
//    }

//    if(kinodynamic_constraints_.max_acc_.rows() != nAx)
//    {
//      CNR_ERROR(logger_,"To compute a trajectory with spline_order_ == spline_order_t::ONE, the accelerations constraint must be defined");
//      return false;
//    }

//    double joint_time, slowest_joint_time;
//    for(size_t iPnt=1; iPnt<path_.size(); iPnt++)
//    {
//      slowest_joint_time = 0.0;
//      for(size_t iAx=0; iAx<nAx; iAx++)
//      {
//        joint_time = std::abs((path_.at(iPnt)[iAx]-path_.at(iPnt-1)[iAx])/kinodynamic_constraints_.max_vel_[iAx]);
//        if(joint_time>slowest_joint_time)
//          slowest_joint_time = std::move(joint_time);
//      }

//      TrjPoint pnt;
//      pnt.state_.pos_.resize(nAx);
//      pnt.state_.vel_.resize(nAx);

//      for(size_t iAx=0; iAx<nAx; iAx++)
//      {
//        pnt.state_.pos_[iAx] = path_.at(iPnt)[iAx];
//        pnt.state_.vel_[iAx] = (path_.at(iPnt)[iAx]-path_.at(iPnt-1)[iAx])/slowest_joint_time;
//      }
//      pnt.time_from_start_ = trj_.back().time_from_start_ + slowest_joint_time;
//      trj_.push_back(pnt);
//    }
//  }
//}


bool SplineTrajectoryProcessor::interpolate(const double& time, TrjPointPtr& pnt, const double& scaling)
{
  if(trj_.empty())
  {
    CNR_ERROR(logger_,"Trajectory is empty or it has not been computed yet");
    return false;
  }

  if((time-trj_.at(0).time_from_start_)<0)
  {
    pnt=trj_.at(0);
    pnt->state_->effort_.resize(trj_.at(0).state_.pos_.size(),0);
    return false;
  }

  if ((time-trj_.back().time_from_start_)>=0)
  {
    pnt=trj_.back();
    pnt->state_.effort_.resize(trj_.back().state_.pos_.size(),0);
    return true;
  }

  for (unsigned int iPnt=1;iPnt<trj_.size();iPnt++)
  {
    if(((time-trj_.at(iPnt).time_from_start_)<0) && ((time-trj_.at(iPnt-1).time_from_start_)>=0))
    {
      unsigned int nAx=trj_.at(iPnt).state_.pos_.size();
      pnt->state_->pos_.resize(nAx,0);
      pnt->state_->vel_.resize(nAx,0);
      pnt->state_->acc_.resize(nAx,0);
      pnt->state_->effort_.resize(nAx,0);
      pnt->time_from_start_=time;
      double delta_time=std::max(1.0e-6,(trj_.at(iPnt).time_from_start_-trj_.at(iPnt-1).time_from_start_));
      double t=(time-trj_.at(iPnt-1).time_from_start_);
      double ratio=t/delta_time;
      for (unsigned int iAx=0;iAx<nAx;iAx++)
      {
        //spline
        if(spline_order_==spline_order_t::ZERO)
        {
          pnt->state_->pos_.at(iAx)=trj_.at(iPnt-1).state_.pos_.at(iAx)+ratio*(trj_.at(iPnt).state_.pos_.at(iAx)-trj_.at(iPnt-1).state_.pos_.at(iAx));
          pnt->state_->vel_.at(iAx)=(trj_.at(iPnt).state_.pos_.at(iAx)-trj_.at(iPnt-1).state_.pos_.at(iAx))/delta_time;
        }
        else if (spline_order_==spline_order_t::ONE)
        {
          double& p0_1=trj_.at(iPnt-1).state_.pos_.at(iAx);
          double& p0_2=trj_.at(iPnt-1).state_.vel_.at(iAx);
          double& pf_1=trj_.at(iPnt).state_.pos_.at(iAx);
          double& pf_2=trj_.at(iPnt).state_.vel_.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = -1.0/(delta_time*delta_time)*(p0_1*3.0-pf_1*3.0+delta_time*p0_2*2.0+delta_time*pf_2);
          double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0-pf_1*2.0+delta_time*p0_2+delta_time*pf_2);

          pnt->state_->pos_.at(iAx) = c1+c2*t+c3*(t*t)+c4*(t*t*t);
          pnt->state_->vel_.at(iAx) = c2+c3*t*2.0+c4*(t*t)*3.0;
          pnt->state_->acc_.at(iAx) = c3*2.0+c4*t*6.0;
        }
        else if (spline_order_==spline_order_t::TWO)
        {
          double& p0_1=trj_.at(iPnt-1).state_.pos_.at(iAx);
          double& p0_2=trj_.at(iPnt-1).state_.vel_.at(iAx);
          double& p0_3=trj_.at(iPnt-1).state_.acc_.at(iAx);
          double& pf_1=trj_.at(iPnt).state_.pos_.at(iAx);
          double& pf_2=trj_.at(iPnt).state_.vel_.at(iAx);
          double& pf_3=trj_.at(iPnt).state_.acc_.at(iAx);

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0E1-pf_1*2.0E1+delta_time*p0_2*1.2E1+delta_time*pf_2*8.0+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3)*(-1.0/2.0);
          double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*3.0E1-pf_1*3.0E1+delta_time*p0_2*1.6E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*2.0)*(1.0/2.0);
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E1-pf_1*1.2E1+delta_time*p0_2*6.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*(-1.0/2.0);

          pnt->state_->pos_.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t);
          pnt->state_->vel_.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0;
          pnt->state_->acc_.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1;
        }
        else if (spline_order_==spline_order_t::THREE)
        {
          double& p0_1=trj_.at(iPnt-1).state_.pos_.at(iAx);
          double& p0_2=trj_.at(iPnt-1).state_.vel_.at(iAx);
          double& p0_3=trj_.at(iPnt-1).state_.acc_.at(iAx);
          double& pf_1=trj_.at(iPnt).state_.pos_.at(iAx);
          double& pf_2=trj_.at(iPnt).state_.vel_.at(iAx);
          double& pf_3=trj_.at(iPnt).state_.acc_.at(iAx);
          // initial and final jerks set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 0.0;
          double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E1-pf_1*1.4E1+delta_time*p0_2*8.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3*2.0-(delta_time*delta_time)*pf_3)*(-5.0/2.0);
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*8.4E1-pf_1*8.4E1+delta_time*p0_2*4.5E1+delta_time*pf_2*3.9E1+(delta_time*delta_time)*p0_3*1.0E1-(delta_time*delta_time)*pf_3*7.0);
          double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E2-pf_1*1.4E2+delta_time*p0_2*7.2E1+delta_time*pf_2*6.8E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.3E1)*(-1.0/2.0);
          double c8 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.0E1-pf_1*1.0E1+delta_time*p0_2*5.0+delta_time*pf_2*5.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*2.0;

          pnt->state_->pos_.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t);
          pnt->state_->vel_.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0;
          pnt->state_->acc_.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1;
        }
        else if (spline_order_==spline_order_t::FOUR)
        {
          double& p0_1=trj_.at(iPnt-1).state_.pos_.at(iAx);
          double& p0_2=trj_.at(iPnt-1).state_.vel_.at(iAx);
          double& p0_3=trj_.at(iPnt-1).state_.acc_.at(iAx);
          double& pf_1=trj_.at(iPnt).state_.pos_.at(iAx);
          double& pf_2=trj_.at(iPnt).state_.vel_.at(iAx);
          double& pf_3=trj_.at(iPnt).state_.acc_.at(iAx);
          // initial and final jerks and spans set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 0.0;
          double c5 = 0.0;
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*3.6E1-pf_1*3.6E1+delta_time*p0_2*2.0E1+delta_time*pf_2*1.6E1+(delta_time*delta_time)*p0_3*5.0-(delta_time*delta_time)*pf_3*3.0)*(-7.0/2.0);
          double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E2-pf_1*1.2E2+delta_time*p0_2*6.4E1+delta_time*pf_2*5.6E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.1E1)*(7.0/2.0);
          double c8 = -1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*5.4E2-pf_1*5.4E2+delta_time*p0_2*2.8E2+delta_time*pf_2*2.6E2+(delta_time*delta_time)*p0_3*6.3E1-(delta_time*delta_time)*pf_3*5.3E1);
          double c9 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.26E2-pf_1*1.26E2+delta_time*p0_2*6.4E1+delta_time*pf_2*6.2E1+(delta_time*delta_time)*p0_3*1.4E1-(delta_time*delta_time)*pf_3*1.3E1)*(5.0/2.0);
          double c10 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*2.8E1-pf_1*2.8E1+delta_time*p0_2*1.4E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*3.0)*(-5.0/2.0);

          pnt->state_->pos_.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t)+c9*(t*t*t*t*t*t*t*t)+c10*(t*t*t*t*t*t*t*t*t);
          pnt->state_->vel_.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0+c9*(t*t*t*t*t*t*t)*8.0+c10*(t*t*t*t*t*t*t*t)*9.0;
          pnt->state_->acc_.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1+c9*(t*t*t*t*t*t)*5.6E1+c10*(t*t*t*t*t*t*t)*7.2E1;
        }
        pnt->state_->vel_.at(iAx)    *= scaling ;
        pnt->state_->acc_.at(iAx) *= scaling*scaling;
      }
      break;
    }
  }
  return true;
}

bool SplineTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
{
  trj_.clear();
  path_ = nullptr;
  logger_ = logger;
  param_ns_ = std::move(param_ns);
  kinodynamic_constraints_ = constraints;

  int spline_order;
  std::string what, full_param_name = param_ns+"/spline_order";
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, spline_order, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      return false;
    }
    else
    {
      switch(spline_order)
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
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }

  return true;
}
bool SplineTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path)
{
  trj_.clear();
  path_ = path;
  logger_ = logger;
  param_ns_ = std::move(param_ns);
  kinodynamic_constraints_ = constraints;

  int spline_order;
  std::string what, full_param_name = param_ns+"/spline_order";
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, spline_order, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      return false;
    }
    else
    {
      switch(spline_order)
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
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }

  return true;
}
}
