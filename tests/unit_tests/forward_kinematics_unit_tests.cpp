/*!
 * @file forward_kinematics_unit_tests.cpp
 *
 * @brief Tests for Aliengo forward kinematics functions
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>

#include "aliengo.hpp"

#include "robcogen/rbd.h"

//! Robot urdf in string format.
std::string robot_urdf{aliengolib::readURDFifstream(aliengolib::aliengo_urdf_path)};

/*!
 * @brief Test for Aliengo::forwardKinematics function
 */
TEST(ForwardKinematics, forward_kinematics)
{
  std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);

  Eigen::VectorXd q_input_eigen{};
  q_input_eigen.setZero(12);
  
  robotlib::RobotBase::JointState q_input{robot->makeJointState(0)};

  Eigen::Matrix<double, 3, 1> default_value{};
  default_value.setZero();

  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
  q_input_eigen << -0.00685785, 0.735254, -1.55269, 0.00561501, 0.892426, -1.56661, -0.014925, 0.903532, -1.57601, -0.00918953, 0.74764, -1.56549;
  foot_position_gt["LF"] = Eigen::Vector3d{0.254555, 0.136442, -0.355861};
  foot_position_gt["RF"] = Eigen::Vector3d{0.201316, -0.132021, -0.352646};
  foot_position_gt["LH"] = Eigen::Vector3d{-0.280547, 0.139218, -0.349002};
  foot_position_gt["RH"] = Eigen::Vector3d{-0.227458, -0.137252, -0.353494};

  for (auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      std::string joint_name{joint->getName()};
      const int joint_id{aliengolib::glue_joint_names_to_ids.at(joint_name)};
      q_input[joint] = q_input_eigen[joint_id];
    }
  }

  robot->forwardKinematics(q_input, foot_position);

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