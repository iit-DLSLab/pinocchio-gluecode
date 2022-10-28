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


std::string robot_urdf{aliengolib::readURDFifstream(aliengolib::aliengo_urdf_path)};

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

  q_input_eigen << 0.0649701, 0.79836, -1.56836, -0.1041, 1.18593, -2.28241, 0.0744313, 1.17117, -2.24656, -0.070271, 0.813742, -1.58492;
  qd_input_eigen << 0.260245, 0.478024, -0.0191248, 1.05619, -3.54174, 0.156615, -2.78084, -4.54272, 1.75207, -0.231391, 0.263121, 0.412205;
  qdd_input_eigen << 6.91327, -5.57871, 11.6183, 39.8007, -149.845, 171.308, -16.6825, -144.333, 176.204, -6.94739, -1.37811, 3.37031;
  gravity_base_input << 0, 0, 0, 0.0379117, -0.587939, -9.79229;
  robot_velocity_input << 0, 0, 0, 0, 0, 0;
  robot_acceleration_input << 16.7327, -2.27715, 0.0866527, -0.226716, 0.143455, 0.945257;
  inv_dyn_tau_gt_eigen << -1.51536, 0.46064, -0.32108, 1.05914, -2.1173, 0.800725, -0.796451, -1.64165, 0.421429, -0.625768, 0.341563, -0.225973;
  wrench_base_gt << 4.74658, -3.76107, -0.77809, 5.8077, 23.3656, 235.177;

  // ** Set robotlib input and ground truth variables
  for (auto leg : *robot->getLegs())
  {
    for(auto joint : *leg->getJoints())
    {
      std::string joint_name{joint->getName()};
      const int joint_id{aliengolib::glue_joint_names_to_ids.at(joint_name)};

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