#ifndef KUKA_IK_SOLVER_HPP
#define KUKA_IK_SOLVER_HPP

#include <memory>
#include <ik_solver_core/ik_solver_base_class.h>
#include <opw_kinematics/opw_parameters.h>

// #define TOLERANCE 1e-3
namespace ik_solver
{

template<typename T>
opw_kinematics::Parameters<T> makeKukaKr5R2500()
{
  opw_kinematics::Parameters<T> p;
  p.a1 = T(0.175);
  p.a2 = T(-0.05);
  p.b =  T(0.0);
  p.c1 = T(0.575);
  p.c2 = T(1.290);
  p.c3 = T(1.035);
  p.c4 = T(0.167);

  p.offsets[1] = -M_PI_2;
  p.sign_corrections[0] = -1;
  p.sign_corrections[3] = -1;
  p.sign_corrections[5] = -1;

  return p;
};

class KukaIkSolver : public IkSolver
{
public:
  virtual bool config(const std::string& param_ns = "") override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;

  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

protected:

  constexpr static unsigned int N_JOINTS {6};
  constexpr static unsigned int N_SOLS    {8};

  opw_kinematics::Parameters<double> opw_params_;
};
}  //  namespace ik_solver


#endif // KUKA_IK_SOLVER_HPP
