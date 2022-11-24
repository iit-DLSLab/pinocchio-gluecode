/*!
 * @file aliengo_model_test.cpp
 *
 * @brief Tests for Aliengo model and robot main functions
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <robotlib/robot_base.hpp>
#include "aliengo.hpp"

#include <gtest/gtest.h>
#include <memory>

//! Robot urdf in string format.
std::string robot_urdf{aliengolib::readURDFifstream(aliengolib::aliengo_urdf_path)};

/*!
 * @brief Test that prints the Aliengo model structure
 */
TEST(AliengoUnitTests, aliengoModel)
{
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);

    auto q = aliengo->makeJointState();

    for (auto leg : *aliengo->getLegs())
    {
        for (auto link : *(leg->getLinks()))
        {
            std::cout << link->getName() << std::endl;
        }
    }
}

/*!
 * @brief Test that prints the Aliengo mass
 */
TEST(RobotBaseUnitTests, getRobotMass)
{
    std::shared_ptr<robotlib::RobotBase> aliengo = createRobotWithUrdf_t(robot_urdf);
    std::cout << aliengo->getRobotMass() << std::endl;
}

/*!
 * @brief Test that calls the inverse dynamics on Aliengo parameters
 */
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

    aliengo->inverseDynamics(v, a, g, q, dq, ddq, wrench_base, tau);
}

/*!
 * @brief Test that prints the Aliengo model data maps and jacobian structures
 */
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