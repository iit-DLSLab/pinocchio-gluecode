/*
 * RobotLenghts.h
 *
 *  Created on: May 10, 2017
 *      Author: Victor Barasuol
 *
 * THIS SHOULDNT EXIST
 */

#ifndef ALIENGO_IK_ROBOTLENGTHS_IK_H_
#define ALIENGO_IK_ROBOTLENGTHS_IK_H_

#include <iostream>

namespace iit{
namespace Aliengo{

/**
 * @brief The RobotLengths class contains the relevant robot lenghts required
 * by kinematics objects (e.g., inverse kinematics), such as the distance
 * between the robot center and the HAA axis.
 * @deprecated This class is deprecated, and included for backcompatibility
 * only. It will be removed soon in the next software release.
 * Please use iit::Aliengo::DefaultParamsGetter instead.
 * @sa iit::Aliengo::DefaultParamsGetter
 */
class RobotLengths {
public:
    RobotLengths() {};
    ~RobotLengths() {};
    inline double get_haa_x() const {
        return 0.2399; // Updated to Aliengo: <xacro:property name="leg_offset_x" value="0.2399"/>
    }
    inline double get_haa_y() const {
        return 0.051; // Updated to Aliengo: <xacro:property name="leg_offset_y" value="0.051"/>
    }
    inline double get_haa_z() const {
        return 0.0; //TODO check if this parameters must be assigned with <xacro:property name="trunk_offset_z" value="0.01675"/>; 
    }
    inline double get_haa_hfe_dist() const {
        return 0.083; // Updated to Aliengo: <xacro:property name="hip_offset" value="0.083"/>
    }
    inline double get_upleg_length() const {
        return 0.25; // Updated to Aliengo: <xacro:property name="thigh_length" value="0.25"/>
    }
    inline double get_lowleg_length() const {
        return 0.25; // Updated to Aliengo: <xacro:property name="calf_length" value="0.25"/>
    }
    inline double get_foot_x() const {
        return 0.25; // Updated to Aliengo: <xacro:property name="calf_length" value="0.25"/>
    }
    inline double get_foot_y() const {
        return 0.0; // Updated to Aliengo
    }
};

}
}

#endif
