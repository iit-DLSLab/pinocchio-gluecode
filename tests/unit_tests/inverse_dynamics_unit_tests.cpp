/**
 * @file arm_unit_tests.cpp
 * @brief Unit tests for Arm class
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>
#include "robotlib/robot_base.hpp"
#include "aliengolib/aliengo.hpp"

///! Robot urdf in string format.
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

/**
 * @brief Set of unit tests for Aliengo::inverse_dynamics function
 */
// TEST(InverseDynamics, inverse_dynamics)
// {
//   /**
//    * @test
//    */

//   // ** Create robot **
//   std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);
  
//   // ** Inputs **
//   robotlib::JointState q_input = robot->makeJointState();	
//   robotlib::JointState qd_input = robot->makeJointState();	
//   robotlib::JointState qdd_input = robot->makeJointState();
//   Eigen::Matrix<double, 6, 1> robot_velocity_input;
//   Eigen::Matrix<double, 6, 1> robot_acceleration_input;
//   Eigen::Matrix<double, 6, 1> gravity_base_input;

//   // ** Outputs ** 
//   robotlib::JointState inv_dyn_tau_output = robot->makeJointState();
//   Eigen::Matrix<double, 6, 1> wrench_base_output;

//   // ** Ground truth** 
//   robotlib::JointState inv_dyn_tau_gt = robot->makeJointState();
//   Eigen::Matrix<double, 6, 1> wrench_base_gt;
    
//   // ** Variable initialization **
//   robot_velocity_input.setZero();
//   robot_acceleration_input.setZero();
//   gravity_base_input.setZero();

//   wrench_base_output.setZero();
  
//   inv_dyn_tau_gt.setZero();
//   wrench_base_gt.setZero();
  
//   gravity_base_input.setZero();

//   // ** Set inputs **
//   // q, qd ,qdd
//   Eigen::VectorXd q_input_eigen(12);
//   Eigen::VectorXd qd_input_eigen(12);        
//   Eigen::VectorXd qdd_input_eigen(12);
//   q_input_eigen << -0.00580286, 0.85555, -1.73281, 0.0348046, 0.874672, -1.86602, -0.0352031, 0.451663, -0.955626, -0.0113642, 0.56335, -1.13148;
//   qd_input_eigen << 0.264297, 0.148724, -0.0441949, -0.272708, 0.132575, -0.018484, 0.2509, 0.0665734, 0.038451, -0.255366, 0.0317115, 0.0929659;
//   qdd_input_eigen << -3.13674, -0.257217, -2.38265, 2.839, -0.620207, -1.31238, -3.47384, -2.48224, 1.45238, 3.41307, -3.59615, 4.21272;

//   // robot velocity
//   robot_velocity_input << 0, 0, 0, 0, 0, 0;

//   // robot acceleration
//   robot_acceleration_input << -3.03244, 1.13813, 0.248626, 0.0366928, -0.0432443, 0.0427975;

//   // gravity_base_input
//   gravity_base_input[3] = -0.0746593;
//   gravity_base_input[4] = 0.0539461;
//   gravity_base_input[5] = -9.80957;

//   // ** Set ground truth **
//   Eigen::VectorXd inv_dyn_tau_gt_eigen(12);
//   inv_dyn_tau_gt_eigen << -0.980625, 0.39525, -0.230276, -0.94224, 0.392556, -0.252692, -1.05742, 0.208105, -0.154878, -1.00744, 0.26486, -0.167876;
//   wrench_base_gt << 0.469748, -2.22008, 0.183519, 2.57093, -1.93818, 211.41;

//   // ** Set robotlib input and ground truth variables
//   for (auto leg : *robot->getLegs())
//   {
//     for(auto joint : *leg->getJoints())
//     {
//       std::string joint_name{joint->getName()};
//       const int joint_id{aliengolib::glue_joint_names_to_ids.at(joint_name)};

//       q_input[joint] = q_input_eigen[joint_id];
//       qd_input[joint] = qd_input_eigen[joint_id];
//       qdd_input[joint] = qdd_input_eigen[joint_id];
//       inv_dyn_tau_gt[joint] = inv_dyn_tau_gt_eigen[joint_id];
//     }
//   }
 
//   // ** Computing the inverse dynamics **
//   robot->inverseDynamics(robot_velocity_input, robot_acceleration_input, gravity_base_input, q_input, qd_input, qdd_input, wrench_base_output, inv_dyn_tau_output);

//   aliengo->inverseDynamics(pose, robot_velocity_input, robot_acceleration_input, q_input, qd_input, qdd_input, inv_dyn_tau_output);
  
//   // ** Test outputs with ground truth ** //TODO
//   // setting threshold
//   double error_th = pow(10,-5);

//   // Testing joint torques
//   for(auto leg : *robot->getLegs())
//   {
//       for (auto joint : *leg->getJoints())
//       {
//         EXPECT_LE(abs(inv_dyn_tau_gt[joint]-inv_dyn_tau_output[joint]),error_th);
//       }
//   }

//   // testing base wrench
//   // error_th = ?;
//   // for (int i=0; i<wrench_base_gt.size(); i++)
//   // {
//   //      EXPECT_LE(abs(wrench_base_gt[i]-wrench_base_output[i]),error_th);
//   // }

// }

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}