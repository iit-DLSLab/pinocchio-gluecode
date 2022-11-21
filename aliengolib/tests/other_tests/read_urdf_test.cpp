/**
 * @file read_urdf_test.cpp
 * @brief Test to verify the reading of the Aliengo URDF
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 */

#include <gtest/gtest.h>

#include <fstream>

/**
 * @brief Test that verifies if the Aliengo URDF is correctly read
 */
TEST(AliengoUnitTests, read_urdf_ifstream)
{
    std::ifstream myfile{"../aliengo.urdf"};
    std::stringstream ss;
    ss << myfile.rdbuf();
    std::string robot_description{ss.str()};
    std::cout << robot_description;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
