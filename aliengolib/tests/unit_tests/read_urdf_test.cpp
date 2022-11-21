/*!
 * @file read_urdf_test.cpp
 * @brief Test to verify the reading of the Aliengo URDF
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include "utils.hpp"

#include <gtest/gtest.h>
#include <fstream>

/*!
 * @brief Test that verifies if the Aliengo URDF is correctly read.
 */
TEST(AliengoUnitTests, read_urdf_ifstream)
{
    std::string robot_urdf{aliengolib::readURDFifstream(aliengolib::aliengo_urdf_path)};
    ASSERT_NE(robot_urdf, "");
}
