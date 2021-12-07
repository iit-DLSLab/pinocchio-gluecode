
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

  robotlib::RobotBase::JointState q = robot->makeJointState();	

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}