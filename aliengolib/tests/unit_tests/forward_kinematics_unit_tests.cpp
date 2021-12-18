
#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>

#include "aliengo.hpp"

#include "robcogen/rbd.h"


//TODO: improvements --> fix the path!
std::string robot_urdf{aliengolib::readURDFifstream("/usr/include/urdf_robots/aliengo.urdf")};

/**
 * @brief Set of unit tests for Aliengo::forwardKinematics
 * The ground truth values are taken executing the RCF controller of the aliengo_dev branch, after the initRFC command
 * commit: dls-distro --> 897b0427400f8708b99fe30775eaa88d00ccc63b
 */
TEST(ForwardKinematics, forward_kinematics)
{
  std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);

  Eigen::VectorXd q_gt_eigen{};
  Eigen::VectorXd qd_gt_eigen{};
  q_gt_eigen.setZero(12);
  qd_gt_eigen.setZero(12);
  
  robotlib::RobotBase::JointState q_gt{robot->makeJointState(0)};	
  robotlib::RobotBase::JointState qd_gt{robot->makeJointState(0)};
  robotlib::RobotBase::JointState qdd_gt{robot->makeJointState(0)};

  Eigen::Matrix<double, 3, 1> default_value{};
  default_value.setZero();

  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_velocity_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_acceleration_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_velocity{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_acceleration{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
  q_gt_eigen << -0.0549458, 0.693107, -1.3977, -0.0533997, 0.691101, -1.39954, -0.0406505, 0.694812, -1.40255, -0.0348423, 0.688963, -1.40672;
  qd_gt_eigen<< -2.88844e-05, 0.00131097, -0.00246203, -2.08106e-05, 0.00127811, -0.00240417, -2.44747e-05, 0.00119205, -0.00224293, -1.30026e-05, 0.00115763, -0.00219036;

  foot_position_gt["LF"] = Eigen::Vector3d{0.242098, 0.154897, -0.377649};
  foot_position_gt["RF"] = Eigen::Vector3d{0.243215, -0.154296, -0.377506};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.237432, 0.149456, -0.378314};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.234409, -0.147232, -0.378174};

  for (auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      std::string joint_name{joint->getName()};
      const int joint_id{aliengolib::glue_joint_names_to_ids[joint_name]};

      q_gt[joint] = q_gt_eigen[joint_id];
      qd_gt[joint] = qd_gt_eigen[joint_id];
    }
  }

  robot->forwardKinematics(q_gt, qd_gt, qdd_gt, foot_position, foot_velocity, foot_acceleration);
  
  double error_th = pow(10,-5);
  for(auto leg : *robot->getLegs())
  {
      Eigen::Vector3d position_gt = foot_position_gt[leg];
      Eigen::Vector3d position_actual = foot_position[leg];

      for (int i=0; i<position_gt.size(); i++)
      {
        EXPECT_LE(abs(position_gt[i]-position_actual[i]),error_th);
      }
  }

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}