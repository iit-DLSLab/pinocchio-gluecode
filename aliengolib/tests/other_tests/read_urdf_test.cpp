#include <gtest/gtest.h>

#include "pugixml/pugixml.hpp"

TEST(AliengoUnitTests, aliengoModel)
{
    // Create empty XML document within memory
    pugi::xml_document doc;
    // Load XML file into memory
    // Remark: to fully read declaration entries you have to specify
    // "pugi::parse_declaration"
    pugi::xml_parse_result result = doc.load_file("../include/aliengo.urdf");//,pugi::parse_default|pugi::parse_declaration)
    if (!result){
        std::cout << "error while loading the aliengo urdf: " << result.description() << std::endl; 
    }
    std::stringstream ss;
    doc.save(ss," ");
    std::string robot_description{ss.str()};
    std::cout << robot_description << std::endl;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
