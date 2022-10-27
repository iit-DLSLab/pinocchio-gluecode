
#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>

#include "aliengo.hpp"

#include "robcogen/rbd.h"


std::string robot_urdf{aliengolib::readURDFifstream(aliengolib::aliengo_urdf_path)};

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
  foot_position_gt["LF"] = Eigen::Vector3d{0.203, 0.139, -0.348};
  foot_position_gt["RF"] = Eigen::Vector3d{0.24, -0.123, -0.37};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.24, 0.145, -0.353};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.279, -0.134, -0.343};
  foot_velocity_gt["LF"] = Eigen::Vector3d{0.0243, -0.0759, 1.86};
  foot_velocity_gt["RF"] = Eigen::Vector3d{-0.225, -0.12, 0.0325};
  foot_velocity_gt["LH"] = Eigen::Vector3d{-0.227, -0.118, -0.0353};
  foot_velocity_gt["RH"] = Eigen::Vector3d{0.0243, -0.0736, 1.92};
  foot_acceleration_gt["LF"] = Eigen::Vector3d{0.382, -3.43, 100};
  foot_acceleration_gt["RF"] = Eigen::Vector3d{-2.29, -5.15, 0.702};
  foot_acceleration_gt["LH"] = Eigen::Vector3d{-1.78, -4.16, -0.964};
  foot_acceleration_gt["RH"] = Eigen::Vector3d{0.418, -3.23, 106};
  // q_gt_eigen << -0.015, 0.897, -1.58, 0.0308, 0.744, -1.49, -0.0299, 0.78, -1.56, 0.000383, 0.923, -1.62;
  // qd_gt_eigen << 0.138, 5.74, -10.5, -0.323, 0.64, -0.0542, 0.335, 0.627, 0.0231, -0.213, 5.79, -10.5;
  // qdd_gt_eigen << 6.86, 291, -513, -14, 4.85, 2.8, 11.7, 4.86, 0.264, -11.4, 302, -527;

  robot->inverseKinematics(foot_position_gt, foot_velocity_gt, foot_acceleration_gt, q, qd, qdd);

  // ** SECOND CALL **
  // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
  foot_position_gt["LF"] = Eigen::Vector3d{0.203, 0.139, -0.34};
  foot_position_gt["RF"] = Eigen::Vector3d{0.239, -0.123, -0.37};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.241, 0.144, -0.353};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.279, -0.134, -0.334};
  foot_velocity_gt["LF"] = Eigen::Vector3d{0.0281, -0.081, 2.21};
  foot_velocity_gt["RF"] = Eigen::Vector3d{-0.234, -0.133, 0.0332};
  foot_velocity_gt["LH"] = Eigen::Vector3d{-0.234, -0.129, -0.0364};
  foot_velocity_gt["RH"] = Eigen::Vector3d{0.0284, -0.0778, 2.29};
  foot_acceleration_gt["LF"] = Eigen::Vector3d{0.959, -1.27, 87.8};
  foot_acceleration_gt["RF"] = Eigen::Vector3d{-2.12, -3.34, 0.174};
  foot_acceleration_gt["LH"] = Eigen::Vector3d{-1.69, -2.57, -0.289};
  foot_acceleration_gt["RH"] = Eigen::Vector3d{1.02, -1.05, 92.1};
  q_gt_eigen << -0.0146785, 0.921859, -1.6283, 0.0298207, 0.747017, -1.48914, -0.0282274, 0.783012, -1.55983, -2.92916e-17, 0.949667, -1.66626;
  qd_gt_eigen << 0.142277, 6.70135, -12.1732, -0.358866, 0.656499, -0.040092, 0.365631, 0.649353, 0.0177547, -0.232934, 6.80512, -12.2078;
  qdd_gt_eigen << 0.242238, 240.216, -416.172, -9.06562, 3.8756, 3.79685, 7.25086, 5.22556, -0.709715, -4.71288, 249.975, -423.288;



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