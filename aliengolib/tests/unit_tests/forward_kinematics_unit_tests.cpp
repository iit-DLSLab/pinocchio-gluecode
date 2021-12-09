
#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>

#include "aliengo.hpp"

#include "robcogen/rbd.h"

/**
 * @brief Set of unit tests for Arm::method_name function
 */
TEST(ForwardKinematics, forward_kinematics)
{
    /**
      * @test
      */
  std::shared_ptr<robotlib::RobotBase> robot = createRobot_t();


  robotlib::RobotBase::JointState q_gt{robot->makeJointState()};	
  robotlib::RobotBase::JointState qd_gt{robot->makeJointState()};
  robotlib::RobotBase::JointState qdd_gt{robot->makeJointState()};

  Eigen::Matrix<double, 3, 1> default_value{};
  default_value.setZero();
  std::cout << default_value << std::endl;
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_velocity_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_acceleration_gt{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_position{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_velocity{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};
  robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> foot_acceleration{robot->makeLegDataMap<Eigen::Matrix<double, 3, 1>>(default_value)};

  robot->forwardKinematics(q_gt, qd_gt, qdd_gt, foot_position, foot_velocity, foot_acceleration);

  for(auto leg : *robot->getLegs())
  {
    std::cout <<foot_position_gt[leg] << std::endl;
    std::cout <<foot_position[leg] << std::endl;
     
      EXPECT_EQ(foot_position_gt[leg],foot_position[leg]);
      EXPECT_EQ(foot_velocity_gt[leg],foot_velocity[leg]);
      EXPECT_EQ(foot_acceleration_gt[leg],foot_acceleration[leg]);
  }

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}