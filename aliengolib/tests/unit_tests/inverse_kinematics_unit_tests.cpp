
#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>

#include "aliengo.hpp"

#include "robcogen/rbd.h"


//TODO: improvements --> fix the path!
std::string robot_urdf{aliengolib::readURDFifstream("../aliengo.urdf")};

/**
 * @brief Set of unit tests for Aliengo::inverse_kinematics
 * The ground truth values are taken executing the RCF controller of the aliengo_dev branch, after the initRFC,stw,ictp (three times f) commands and letting aliengo walking on a ramp
 * commit: dls-distro --> 897b0427400f8708b99fe30775eaa88d00ccc63b
 */
TEST(InverseKinematics, inverse_kinematics)
{
  std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);
  
  robot->setInvKinTimePeriod(0.004);

  Eigen::VectorXd q_gt_eigen{};
  Eigen::VectorXd qd_gt_eigen{};
  Eigen::VectorXd qdd_gt_eigen{};
  q_gt_eigen.setZero(12);
  qd_gt_eigen.setZero(12);
  qdd_gt_eigen.setZero(12);
  
  robotlib::RobotBase::JointState q_gt{robot->makeJointState(0)};	
  robotlib::RobotBase::JointState qd_gt{robot->makeJointState(0)};
  robotlib::RobotBase::JointState qdd_gt{robot->makeJointState(0)};
  
  robotlib::RobotBase::JointState q{robot->makeJointState(0)};	
  robotlib::RobotBase::JointState qd{robot->makeJointState(0)};
  robotlib::RobotBase::JointState qdd{robot->makeJointState(0)};

  Eigen::Matrix<double, 3, 1> default_value{};
  default_value.setZero();

  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_velocity_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_acceleration_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  // ** The inverse kinematics tests is based on calling two times the inverse kinematics.**
  // This is because the computation of qdd uses old_feet_jacobians_ variable which depends on
  // the previous call. So in the first call of the inverse kinematics the old_feet_jacobians_ variable is set for the first time.
  // In the second call, we can then evaluate the inverse kinematics, comparing q, qd, and qdd with the ground truth.

  // ** FIRST CALL **
  // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
  // q_gt_eigen   << -0.0125531, 0.996375, -2.00237, 0.00509447, 0.862054, -1.83091, -0.00161759, 0.432273, -0.936659, -0.0223736, 0.707733, -1.39797;
  // qd_gt_eigen  << 0.0454697, 2.18132, -5.03128, -0.201104, 0.330191, -0.00315145, 0.133991, 0.216089, -0.0197947, -0.102764, 3.16978, -6.83554;
  // qdd_gt_eigen << -2.50026, -32.9323, 41.3153, 2.75067, -1.06054, -0.61156, -2.31743, -1.28083, 0.534671, 6.90676, -37.6316, 52.1575;

  foot_position_gt["LF"] = Eigen::Vector3d{0.241196, 0.137378, -0.268587};
  foot_position_gt["RF"] = Eigen::Vector3d{0.256164, -0.132449, -0.3047};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.223816, 0.134721, -0.445737};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.243248, -0.142542, -0.38078};

  foot_velocity_gt["LF"] = Eigen::Vector3d{ 0.0850577, -0.0255141, 1.05563};
  foot_velocity_gt["RF"] = Eigen::Vector3d{-0.100025, -0.0612524, 0.011659};
  foot_velocity_gt["LH"] = Eigen::Vector3d{-0.0920158, -0.0597229, -0.0123018};
  foot_velocity_gt["RH"] = Eigen::Vector3d{ 0.104533, -0.0145508, 1.10783};

  foot_acceleration_gt["LF"] = Eigen::Vector3d{3.96104, 0.865219, -8.86985};
  foot_acceleration_gt["RF"] = Eigen::Vector3d{0.40879, 0.838311, -0.0801642};
  foot_acceleration_gt["LH"] = Eigen::Vector3d{0.454411, 1.03452, 0.138344};
  foot_acceleration_gt["RH"] = Eigen::Vector3d{4.98252, 2.28562, -9.85851};

  robot->inverseKinematics(foot_position_gt, foot_velocity_gt, foot_acceleration_gt, q, qd, qdd);

  // ** SECOND CALL **
  // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
  q_gt_eigen   <<  -0.0123685, 1.00473, -2.022, 0.00432246, 0.863359, -1.83092, -0.00110779, 0.43312, -0.936729, -0.0227353, 0.719903, -1.42451;
  qd_gt_eigen  << 0.0371645, 1.97562, -4.77012, -0.189597, 0.324761, -0.00529355, 0.124344, 0.20893, -0.0151846, -0.0766078, 2.90027, -6.4257;
  qdd_gt_eigen << -2.07607, -41.6222, 55.5532, 2.87858, -1.25218, -0.756396, -2.40483, -1.62629, 0.820005, 7.49685, -48.5074, 70.9453;

  foot_position_gt["LF"] = Eigen::Vector3d{0.241564, 0.137277, -0.264455};
  foot_position_gt["RF"] = Eigen::Vector3d{0.255769, -0.132684, -0.304655};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.224179, 0.134494, -0.445785};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.242795, -0.142582, -0.37645};
  foot_velocity_gt["LF"] = Eigen::Vector3d{0.102373, -0.0223346, 1.00788};
  foot_velocity_gt["RF"] = Eigen::Vector3d{-0.0980735, -0.0577441, 0.0114232};
  foot_velocity_gt["LH"] = Eigen::Vector3d{-0.0898321, -0.0554289, -0.0118346};
  foot_velocity_gt["RH"] = Eigen::Vector3d{ 0.126323, -0.00499331, 1.05567};
  foot_acceleration_gt["LF"] = Eigen::Vector3d{4.32882, 0.794856, -11.9374};
  foot_acceleration_gt["RF"] = Eigen::Vector3d{0.487808, 0.87708, -0.0589506};
  foot_acceleration_gt["LH"] = Eigen::Vector3d{0.545924, 1.07349, 0.116799};
  foot_acceleration_gt["RH"] = Eigen::Vector3d{5.44738, 2.38938, -13.0389};

  for (auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      std::string joint_name{joint->getName()};
      const int joint_id{aliengolib::glue_joint_names_to_ids[joint_name]};

      q_gt[joint] = q_gt_eigen[joint_id];
      qd_gt[joint] = qd_gt_eigen[joint_id];
      qdd_gt[joint] = qdd_gt_eigen[joint_id];
    }
  }
  
  robot->inverseKinematics(foot_position_gt, foot_velocity_gt, foot_acceleration_gt, q, qd, qdd);

  double error_th = pow(10,-3);
  for(auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      EXPECT_LE(abs(q_gt[joint]-q[joint]), error_th);
      EXPECT_LE(abs(qd_gt[joint]-qd[joint]), error_th);

      EXPECT_LE(abs(qdd_gt[joint]-qdd[joint]), error_th);
    }
  }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}