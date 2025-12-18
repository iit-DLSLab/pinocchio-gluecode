#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include "robotlib/robot_factory.hpp"

// Small demo
int main()
{
    std::shared_ptr<robotlib::RobotBase> pRobot;
    const std::string robot_name{"aliengo"};
    try
    {
        pRobot = robotlib::RobotFactory::openRobot(robot_name);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Could not open the robot " << robot_name << std::endl;
        std::cerr << e.what() << std::endl;
    }
    return 0;
}