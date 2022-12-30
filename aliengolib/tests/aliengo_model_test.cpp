#include "robotlib/robot_base.hpp"
#include "aliengolib/aliengo.hpp"

#include <gtest/gtest.h>
#include <memory>

//TODO: improvements --> fix the path!
std::string robot_urdf{aliengolib::readURDFifstream("/usr/include/robots/aliengo.urdf")};

TEST(AliengoUnitTests, aliengoModel)
{
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);

    for (auto leg : *aliengo->getLegs())
    {
        for (auto link : *(leg->getLinks()))
        {
            std::cout << link->getName() << std::endl;
        }
    }
}

TEST(RobotBaseUnitTests, getRobotMass)
{
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);
    std::cout << aliengo->getRobotMass() << std::endl;
}

TEST(RobotBaseUnitTests, inverseDynamics)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);

    Eigen::Matrix<double, 6, 1> v;
    Eigen::Matrix<double, 6, 1> a;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 1> wrench_base; ///output

    auto q = aliengo->makeJointState();
    auto dq = aliengo->makeJointState();
    auto ddq = aliengo->makeJointState();
    auto tau = aliengo->makeJointState();

    aliengo->inverseDynamics(wrench_base, tau, g, q, dq, ddq, v, a);
}

TEST(RobotBaseUnitTests, dataMap_constructor_with_initialization)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);
    double data_double{1};
    double data{1};
    typedef double type;
    // typedef Eigen::Vector3d type;
    // Eigen::Vector3d data;
    // data.setOnes();

    auto leg_dm = aliengo->makeLegDataMap<type>(data);
    auto link_dm = aliengo->makeLinkDataMap<type>(data);
    auto joint_dm = aliengo->makeJointDataMap<type>(data);
    auto jacobian_dm = aliengo->makeFeetJacobian(data_double);

    std::cout << "leg_dm\n";
    for (auto data : leg_dm)
    {
        std::cout << data.data_ << " ";
    }
    std::cout << "\n";

    std::cout << "link_dm\n";
    for (auto data : link_dm)
    {
        std::cout << data.data_ << " ";
    }
    std::cout << "\n";

    std::cout << "joint_dm\n";
    for (auto data : joint_dm)
    {
        std::cout << data.data_ << " ";
    }
    std::cout << "\n";

    std::cout << "jacobian_dm\n";
    for (auto leg : *(aliengo->getLegs()))
    {
        std::cout << jacobian_dm[leg] << "\n";
        std::cout << "***\n";
    }
    std::cout << "\n";
}

TEST(RobotBaseUnitTests, getRobotCoM)
{
    /// Dummy quadruped
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);

    std::cout << aliengo->getRobotCoM() << std::endl;
}

/*!
 * @brief Test getting linear and angular foot jacobian. The ground truth are taken from the simulation, launching the robcogen functions without using Robotlib and Aliengolib.
 */
TEST(AliengoUnitTests, getFootJacobian)
{
    std::shared_ptr<robotlib::RobotBase> robot = createRobotWithUrdf_t(robot_urdf);

    robot->setInvKinTimePeriod(0.004);

    Eigen::Vector<double,12> q_eigen{};
    q_eigen.setZero();
    robotlib::JointState q{robot->makeJointState(0)};
    auto feet_jacobian_gt{robot->makeFeetJacobian(0)};

    auto feet_jacobian{robot->makeFeetJacobian(0)};

    // Ground truth values, HP: the order of the values is the same as the one of the joints defined in the glue!
    q_eigen << -0.059668, 0.679341, -1.48335, -0.0474497, 0.932931, -1.5288, 0.0472684, 1.01442, -1.63769, 0.0596151, 0.723763, -1.58137;
    for (auto leg : *robot->getLegs())
    {
        for(auto joint : *leg->getJoints())
        {
        std::string joint_name{joint->getName()};
        const int joint_id{aliengolib::glue_joint_names_to_ids[joint_name]};
        q[joint] = q_eigen[joint_id];
        }
    }

    feet_jacobian_gt["LF"].row(0) << 0, -0.367954, -0.173457;
    feet_jacobian_gt["LF"].row(1) << -0.36235, 0.00136945, 0.010736;
    feet_jacobian_gt["LF"].row(2) << -0.104794, -0.0229239, -0.179715;
    feet_jacobian_gt["LF"].row(3) << -1, 0, 0;
    feet_jacobian_gt["LF"].row(4) << 0, 0.99822, 0.99822;
    feet_jacobian_gt["LF"].row(5) << 0, 0.0596326, 0.0596326;
    feet_jacobian_gt["RF"].row(0) << 0, -0.355786, -0.206916;
    feet_jacobian_gt["RF"].row(1) << 0.351449, 0.00287133, -0.00665499;
    feet_jacobian_gt["RF"].row(2) << -0.0997822, 0.0604678, -0.140148;
    feet_jacobian_gt["RF"].row(3) << 1, 0, 0;
    feet_jacobian_gt["RF"].row(4) << 0, 0.998874, 0.998874;
    feet_jacobian_gt["RF"].row(5) << 0, -0.0474318, -0.0474318;
    feet_jacobian_gt["LH"].row(0) << 0, -0.335021, -0.202993;
    feet_jacobian_gt["LH"].row(1) << -0.338569, 0.00313601, -0.00689503;
    feet_jacobian_gt["LH"].row(2) << -0.0670773, 0.0662953, -0.145761;
    feet_jacobian_gt["LH"].row(3) << -1, 0, 0;
    feet_jacobian_gt["LH"].row(4) << 0, 0.998883, 0.998883;
    feet_jacobian_gt["LH"].row(5) << 0, -0.0472508, -0.0472508;
    feet_jacobian_gt["RH"].row(0) << 0, -0.350892, -0.163562;
    feet_jacobian_gt["RH"].row(1) << 0.355213, 0.0014012, 0.0112648;
    feet_jacobian_gt["RH"].row(2) << -0.0619465, -0.0234763, -0.188734;
    feet_jacobian_gt["RH"].row(3) << 1, 0, 0;
    feet_jacobian_gt["RH"].row(4) << 0, 0.998224, 0.998224;
    feet_jacobian_gt["RH"].row(5) << 0, 0.0595798, 0.0595798;

    double error_th = pow(10,-3);
    for (auto leg : *robot->getLegs())
    {
        robot->getFootJacobian(q, leg, feet_jacobian[leg]);
        for(int i=0; i<6;i++)
        {
            for (int j=0; j<leg->getNJoints(); j++)
            {
                EXPECT_LE(fabs(feet_jacobian_gt[leg](i,j)-feet_jacobian[leg](i,j)), error_th);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}