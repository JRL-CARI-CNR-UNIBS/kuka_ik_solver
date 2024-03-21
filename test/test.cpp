// From fk_ik_tests.cpp of opw_kinematics

#include <opw_kinematics/opw_macros.h>
OPW_IGNORE_WARNINGS_PUSH
#include <array>
#include <chrono>
#include <gtest/gtest.h>  // IWYU pragma: keep
#include <random>
#include <array>
OPW_IGNORE_WARNINGS_POP

#include "opw_kinematics/opw_parameters_examples.h"
#include "opw_kinematics/opw_utilities.h"

#include "kuka_ik_solver/kuka_ik_solver.hpp"

// Structure used for setting sampler parameters based on floating point type
template <typename T>
struct TestTolerance;

template <>
struct TestTolerance<float>
{
  static constexpr float TOLERANCE = 5e-4f;
  static constexpr const char* const NAME = "float";
};

template <>
struct TestTolerance<double>
{
  static constexpr double TOLERANCE = 1e-7;
  static constexpr const char* const NAME = "double";
};

// Helper Functions for generating and testing random poses in cartesian/joint space
using opw_kinematics::Solutions;
using opw_kinematics::Transform;

template <typename T>
void comparePoses(const Transform<T>& Ta, const Transform<T>& Tb, double tolerance)
{
  T translation_distance = (Ta.translation() - Tb.translation()).norm();
  T angular_distance = Eigen::Quaternion<T>(Ta.linear()).angularDistance(Eigen::Quaternion<T>(Tb.linear()));

  EXPECT_NEAR(static_cast<double>(translation_distance), 0, tolerance);
  EXPECT_NEAR(static_cast<double>(angular_distance), 0, tolerance);
}

template <typename T>
void getRandomJointValues(std::array<T, 6>& q)
{
  static std::default_random_engine en{ 42 };                        // random engine
  static std::uniform_real_distribution<T> rg{ T(-M_PI), T(M_PI) };  // random generator

  q[0] = rg(en);
  q[1] = rg(en);
  q[2] = rg(en);
  q[3] = rg(en);
  q[4] = rg(en);
  q[5] = rg(en);
}

// Generate random joint pose -> Solve FK -> Solve IK -> Verify that new FK matches
// the original.
template <typename T>
void runRandomReachablePosesTest(const opw_kinematics::Parameters<T>& params)
{
  const static unsigned int number_of_tests = 1000;

  Transform<T> pose, forward_pose;
  std::array<T, 6> q_rand;
  Solutions<T> sols;

  for (unsigned j = 0; j < number_of_tests; ++j)
  {
    // create a reachable pose based on a random robot position
    getRandomJointValues(q_rand);
    pose = opw_kinematics::forward(params, q_rand);

    // Solve Inverse kinematics
    sols = opw_kinematics::inverse(params, pose);

    // check all valid solutions using forward kinematics
    for (const auto& s : sols)
    {
      if (opw_kinematics::isValid(s))
      {
        // Forward kinematics of a solution should result in the same pose
        forward_pose = opw_kinematics::forward(params, s);
        comparePoses(forward_pose, pose, TestTolerance<T>::TOLERANCE);
      }
    }
  }
}

//template <typename T>
void compareJoints(const std::array<double, 6>& qa, const std::array<double, 6>& qb, double tolerance)
{
  std::cout << "==============================================" << std::endl;
  for(size_t idx = 0; idx < 6; idx++)
  {
    EXPECT_NEAR(qa[idx], qb[idx], tolerance);
  }
}

// Generate random joint poses, time how long it takes to solve FK for all of them
// Then time how long it takes to solve IK for all of them
template <typename T>
void runThroughputTests(const opw_kinematics::Parameters<T>& params)
{
  const static unsigned int number_of_tests = 10000;

  std::vector<Transform<T>, Eigen::aligned_allocator<Transform<T>>> poses;
  poses.resize(number_of_tests);

  {
    // Generate N random joint poses
    std::vector<std::array<T, 6>> random_joint_values;
    random_joint_values.resize(number_of_tests);

    for (std::size_t i = 0; i < number_of_tests; ++i)
    {
      getRandomJointValues(random_joint_values[i]);
    }

    // Time the solving of FK
    const auto fk_start_tm = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < number_of_tests; ++i)
    {
      poses[i] = opw_kinematics::forward(params, random_joint_values[i]);
    }
    const auto fk_end_tm = std::chrono::steady_clock::now();

    // Report FK timing
    const auto fk_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(fk_end_tm - fk_start_tm).count();
    std::cout << "Forward Kinematics " << TestTolerance<T>::NAME << " generated " << number_of_tests << " poses in "
              << fk_dt_us << " us\n";
    std::cout << "Average us per fk solve: " << static_cast<double>(fk_dt_us) / number_of_tests << "\n";
  }

  Solutions<T> ik_sol_space;
  const auto ik_start_tm = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < number_of_tests; ++i)
  {
    ik_sol_space = opw_kinematics::inverse(params, poses[i]);
  }
  const auto ik_end_tm = std::chrono::steady_clock::now();
  // Report FK timing
  const auto ik_dt_us = std::chrono::duration_cast<std::chrono::microseconds>(ik_end_tm - ik_start_tm).count();
  std::cout << "Inverse Kinematics " << TestTolerance<T>::NAME << " generated " << number_of_tests << " poses in "
            << ik_dt_us << " us\n";
  std::cout << "Average us per ik solve: " << static_cast<double>(ik_dt_us) / number_of_tests << "\n";
}

TEST(kuka_kr50r2500, random_reachable_poses_double)  // NOLINT
{
  const auto kukaKR6_R700_sixx = ik_solver::makeKukaKr5R2500<double>();
  runRandomReachablePosesTest(kukaKR6_R700_sixx);
}

TEST(kuka_kr50r2500, random_reachable_poses_float)  // NOLINT
{
  const auto kukaKR6_R700_sixx = ik_solver::makeKukaKr5R2500<float>();
  runRandomReachablePosesTest(kukaKR6_R700_sixx);
}

TEST(kuka_kr50r2500, throughput_tests_float)  // NOLINT
{
  const auto params = ik_solver::makeKukaKr5R2500<float>();
  runThroughputTests(params);
}

TEST(kuka_kr50r2500, throughput_tests_double)  // NOLINT
{
  const auto params = ik_solver::makeKukaKr5R2500<double>();
  runThroughputTests(params);
}

TEST(kuka_kr50r2500, numerical_issue_double)  // NOLINT
{
  const auto params = ik_solver::makeKukaKr5R2500<double>();

  Transform<double> pose2{ Transform<double>::Identity() };
  pose2(0, 0) = -0.707106781186547;
  pose2(1, 0) = 0;
  pose2(2, 0) = -0.707106781186548;
  pose2(0, 1) = 4.9065389333868E-018;
  pose2(1, 1) = 1;
  pose2(2, 1) = -4.9065389333868E-018;
  pose2(0, 2) = 0.707106781186548;
  pose2(1, 2) = 0;
  pose2(2, 2) = -0.707106781186547;
  pose2(0, 3) = 1.3485459941112;
  pose2(1, 3) = 0;
  pose2(2, 3) = 0.399038059222874;

//  comparePoses(pose, pose2, TestTolerance<double>::TOLERANCE);

  auto sols = opw_kinematics::inverse(params, pose2);
  for (const auto& s : sols)
  {
    Transform<double> f1 = opw_kinematics::forward(params, s);
    comparePoses(pose2, f1, TestTolerance<double>::TOLERANCE);
  }
}

TEST(kuka_kr50r2500, compare_joints)
{
  const auto params = ik_solver::makeKukaKr5R2500<double>();

  Eigen::Isometry3d iso;
  iso.matrix() <<  -0.167,  0.966, -0.198,  0.861,
                   -0.131,  0.177,  0.975, -0.524,
                    0.977,  0.189,  0.097,  1.667,
                    0.000,  0.000,  0.000,  1.000;
  std::array<double, 6> g{{0.6457719999999996,
                           -0.060213099999999936,
                           -1.7907124000000008,
                           0.753981599999999,
                           -1.2653627999999992,
                           -2.0e-323}};
  auto sols = opw_kinematics::inverse(params, iso);
  for(const auto& s : sols)
  {
    compareJoints(g, s, TestTolerance<double>::TOLERANCE);
  }
}
