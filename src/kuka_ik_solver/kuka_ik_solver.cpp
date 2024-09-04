#include <kuka_ik_solver/kuka_ik_solver.hpp>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ik_solver::KukaIkSolver, ik_solver::IkSolver)

namespace ik_solver{

bool KukaIkSolver::config(const std::string& param_ns)
{
  if(!IkSolver::config(param_ns))
  {
    return false;
  }

  opw_params_ = makeKukaKr5R2500<double>();
  return true;
}

Solutions KukaIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& /*seeds*/, const int& /*desired_solutions*/, const int& /*min_stall_iterations*/, const int& /*max_stall_iterations*/)
{
  ik_solver::Solutions sols;
  opw_kinematics::Solutions<double> opw_sols;
  Eigen::Isometry3d T_base_flange_iso;
  T_base_flange_iso.translation() = T_base_flange.translation();
  T_base_flange_iso.linear() = T_base_flange.rotation();
  opw_sols = opw_kinematics::inverse(opw_params_, T_base_flange_iso);

  sols.configurations().reserve(8);

  Configurations conf;



  for(auto& sol : opw_sols)
  {
    if(opw_kinematics::isValid(sol))
    {
      // un po' ridondante il crea-distruggi
      conf.push_back(Eigen::VectorXd{{sol.at(0), sol.at(1), sol.at(2), sol.at(3), sol.at(4), sol.at(5)}});
      std::vector<int> out_of_bound = outOfBound(conf.back(), ub_, lb_);
      if (std::find(out_of_bound.begin(), out_of_bound.end(), 0) != out_of_bound.end())
      {
        conf.pop_back();
      }
    }
    else
    {
      continue;
    }
  }

  sols.configurations()=getMultiplicity(conf,ub_,lb_,revolute_);
//  sols.configurations()=conf;

//  std::remove_if(sols.configurations().begin(), sols.configurations().end(), [](const auto& el){
//    return std::isnan(el.norm());
//  });

  sols.translation_residuals().resize(sols.configurations().size());
  sols.rotation_residuals().resize(sols.configurations().size());
  std::fill(sols.translation_residuals().begin(), sols.translation_residuals().end(), std::numeric_limits<double>::epsilon());
  std::fill(sols.rotation_residuals().begin(), sols.rotation_residuals().end(), std::numeric_limits<double>::epsilon());

  for (size_t idx=0;idx<sols.configurations().size();idx++)
  {
      Eigen::Affine3d T_base_flange_iso_cal=getFK(sols.configurations().at(idx));
      sols.translation_residuals().at(idx)=(T_base_flange_iso.translation()-T_base_flange_iso_cal.translation()).norm();
      Eigen::AngleAxisd aa;
      aa=T_base_flange.linear().inverse()*T_base_flange_iso_cal.linear();
      sols.rotation_residuals().at(idx)=aa.angle();
  }

  if(int num = sols.configurations().size() != 0)
    sols.message() = std::to_string(num) + " solutions found";
  else
    sols.message() = "No solution found";

  return sols;
}

Eigen::Affine3d KukaIkSolver::getFK(const Configuration &s)
{
  std::array<double, 6> s_opw;
  for(long idx = 0; idx < s.size(); ++idx)
  {
    s_opw.at(idx) = s(idx);
  }
  return opw_kinematics::forward(opw_params_, s_opw);
}

} // namespace ik_solver
