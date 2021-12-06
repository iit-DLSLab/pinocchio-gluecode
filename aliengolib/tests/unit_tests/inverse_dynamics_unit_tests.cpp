/**
 * @file arm_unit_tests.cpp
 * @brief Unit tests for Arm class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include <robotlib/robot_base.hpp>
#include <robotlib/robot_factory.hpp>

// copy of the rpyToRot function inside rotations.h of aliengolib::commons
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

//TODO: robcogen
//Robcogen
// #include "robcogen/rbd.h"
/**
 * @brief Set of unit tests for Arm::method_name function
 */
// TEST(InverseDynamics, inverse_dynamics_dlopen)
// {
//     /**
//       * @test
//       */
//   std::shared_ptr<robotlib::RobotBase> robot = robotlib::RobotFactory::openRobot("aliengolib");

// 	robotlib::RobotBase::JointState q = robot->makeJointState();	
// 	robotlib::RobotBase::JointState qd = robot->makeJointState();	
// 	robotlib::RobotBase::JointState qdd = robot->makeJointState();	
// 	robotlib::RobotBase::JointState invDynTau = robot->makeJointState();
// 	robotlib::RobotBase::JointState invDynTau_gt = robot->makeJointState();	//computed using id_fully_actuated method of the inverse dynamics

//   aliengolib::rbd::Vector6D gW = aliengolib::rbd::Vector6D::Zero();
// 	aliengolib::rbd::Vector6D gB = aliengolib::rbd::Vector6D::Zero();
  
//   aliengolib::rbd::ForceVector baseWrench;
	
//   gW(aliengolib::rbd::LZ) =  -aliengolib::rbd::g;
// 	gB.setZero();

// 	// Set robot orientation //TODO
//   Eigen::Vector3d rpy;
//   rpy(0) = 0.8; //roll
//   rpy(1) = 0.8; //pitch
//   rpy(2) = 0.8; //yaw
	
//   // Set robot q, qd ,qdd //TODO
//   Eigen::VectorXd q_values(18);
//   Eigen::VectorXd qd_values(18);
//   Eigen::VectorXd qdd_values(18);
//   q_values << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
//   qd_values << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
//   qdd_values << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

// 	// // Set ground truth inverse dynamics //TODO
//   Eigen::VectorXd invDynTau_gt_values(18);
//   invDynTau_gt_values << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

//   // transform gravity from world to base frame
// 	gB.segment(aliengolib::rbd::LX,3) = rpyToRot(aliengolib::rbd::Vector3d(rpy[0], rpy[1], rpy[2]))*gW.segment(aliengolib::rbd::LX,3);


// 	// robot->inverseDynamics(aliengolib::rbd::Vector6D::Zero(), aliengolib::rbd::Vector6D::Zero(), gB, q, qd, qdd, baseWrench, invDynTau);
  
//   // check estimated tau with ground truth //TODO
// }

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}