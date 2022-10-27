/**
 * @file arm_unit_tests.cpp
 * @brief Unit tests for Arm class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>
#include "aliengo.hpp"


std::string robot_urdf{aliengolib::readURDFifstream("/usr/lib/robots/aliengo.urdf")};

// copy of the rpyToRot function inside rotations.h of iit::commons
Eigen::Matrix3d inline rpyToRot(const Eigen::Vector3d & rpy){

    Eigen::Matrix3d Rx, Ry, Rz;
    double roll, pitch, yaw;

    roll = rpy(0);
    pitch = rpy(1);
    yaw = rpy(2);

    Rx <<	1   ,    0     	  ,  	  0,
            0   ,    cos(roll) ,  sin(roll),
            0   ,    -sin(roll),  cos(roll);


    Ry << cos(pitch) 	,	 0  ,   -sin(pitch),
            0       ,    1  ,   0,
            sin(pitch) 	,	0   ,  cos(pitch);

    Rz << cos(yaw)  ,  sin(yaw) ,		0,
            -sin(yaw) ,  cos(yaw) ,  		0,
            0      ,     0     ,       1;


    return Rx*Ry*Rz;

}

//Robcogen
#include "robcogen/rbd.h"
/**
 * @brief Set of unit tests for Aliengo::inverse_dynamics function
 */
TEST(InverseDynamics, inverse_dynamics)
{
  /**
   * @test
   */

  // ** Create robot **
  std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);
  
  // ** Inputs **
  robotlib::RobotBase::JointState q_input = robot->makeJointState();	
  robotlib::RobotBase::JointState qd_input = robot->makeJointState();	
  robotlib::RobotBase::JointState qdd_input = robot->makeJointState();
  Eigen::Matrix<double, 6, 1> robot_velocity_input;
  Eigen::Matrix<double, 6, 1> robot_acceleration_input;
  Eigen::Matrix<double, 6, 1> gravity_base_input;

  // ** Outputs ** 
  robotlib::RobotBase::JointState inv_dyn_tau_output = robot->makeJointState();
  Eigen::Matrix<double, 6, 1> wrench_base_output;

  // ** Ground truth** 
  robotlib::RobotBase::JointState inv_dyn_tau_gt = robot->makeJointState();
  Eigen::Matrix<double, 6, 1> wrench_base_gt;
    
  // ** Variable initialization **
  // robot_velocity_input.setZero();
  // robot_acceleration_input.setZero();
  // gravity_base_input.setZero();

  wrench_base_output.setZero();
  
  // inv_dyn_tau_gt.setZero();
  // wrench_base_gt.setZero();
  
  // gravity_base_input.setZero();

  // ** Set inputs **
  // q, qd ,qdd
  Eigen::VectorXd q_input_eigen(12);
  Eigen::VectorXd qd_input_eigen(12);        
  Eigen::VectorXd qdd_input_eigen(12);
  // ** Set ground truth **
  Eigen::VectorXd inv_dyn_tau_gt_eigen(12);

  q_input_eigen << -0.0935791, 0.804959, -1.58939, -0.0903319, 0.804818, -1.59102, -0.101786, 0.802827, -1.59957, -0.10254, 0.801992, -1.59863;
  qd_input_eigen << -3.61437e-05, 0.00110956, -0.00223296, -0.000104387, 0.00109778, -0.00217754, -2.53862e-05, -7.96748e-05, -2.98041e-05, -3.17601e-05, -6.77181e-05, -1.64018e-05;
  qdd_input_eigen << -2.50802e-06, 1.74728e-05, 3.38871e-06, 2.63621e-06, 1.82047e-05, -3.92071e-07, -2.30064e-07, 1.45463e-05, 9.37137e-06, 1.0833e-06, 1.39333e-05, 7.97995e-06;
  gravity_base_input << 0, 0, 0, -0.0563401, 0.00249432, -9.80984;
  robot_velocity_input << 0, 0, 0, 0, 0, 0;
  robot_acceleration_input << 2.90385e-07, 2.93073e-06, -1.52975e-06, 8.1307e-06, -7.27653e-07, 8.36325e-07;
  inv_dyn_tau_gt_eigen << -1.08885, 0.438185, -0.269171, -1.08629, 0.437753, -0.269693, -1.09519, 0.433331, -0.272091, -1.09592, 0.432766, -0.27203;
  wrench_base_gt << 0.0474523, 1.77105, 0.000175675, 1.40008, -0.0619934, 243.745;

  // ** Set robotlib input and ground truth variables
  for (auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      std::string joint_name{joint->getName()};
      const int joint_id{aliengolib::glue_joint_names_to_ids[joint_name]};

      q_input[joint] = q_input_eigen[joint_id];
      qd_input[joint] = qd_input_eigen[joint_id];
      qdd_input[joint] = qdd_input_eigen[joint_id];
      inv_dyn_tau_gt[joint] = inv_dyn_tau_gt_eigen[joint_id];
    }
  }
 
  // ** Computing the inverse dynamics **
  robot->inverseDynamics(robot_velocity_input, robot_acceleration_input, gravity_base_input, q_input, qd_input, qdd_input, wrench_base_output, inv_dyn_tau_output);

  // ** Test outputs with ground truth **
  double error_th = pow(10,-3);
  // Testing joint torques
  for(auto leg : *robot->getLegs())
  {
      for (auto joint : *leg->getJoints())
      {
        EXPECT_LE(abs(inv_dyn_tau_gt[joint]-inv_dyn_tau_output[joint]),error_th);
      }
  }

  // testing base wrench
  for (int i=0; i<wrench_base_gt.size(); i++)
  {
       EXPECT_LE(abs(wrench_base_gt[i]-wrench_base_output[i]),error_th);
  }

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}