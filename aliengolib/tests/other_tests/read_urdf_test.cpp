#include <gtest/gtest.h>

#include "pugixml/pugixml.hpp"

#include <fstream>

TEST(AliengoUnitTests, aliengoModel)
{
    std::ifstream myfile{"../include/aliengo.urdf"};
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
