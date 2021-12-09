#include <gtest/gtest.h>

#include <fstream>

TEST(AliengoUnitTests, read_urdf_ifstream)
{
    std::ifstream myfile{"/usr/include/urdf_robots/aliengo.urdf"};
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
