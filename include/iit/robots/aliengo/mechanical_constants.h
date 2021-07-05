/*
 * mechanical_constants.h
 *
 *  Created on: Jun 3, 2014
 *      Author: marco
 */

/*****************************************************************************    THIS FILE DOES NOT MAKE SENSE FOR ALIENGO   *****************************************/

#ifndef IIT_ALIENGO_MECHANICAL_CONSTANTS_H_
#define IIT_ALIENGO_MECHANICAL_CONSTANTS_H_

#include <cmath>

namespace iit {
/**
 * Aliengo stuff
 */
namespace Aliengo {

/**
 * Four-bar linkage mechanical constants
 */
namespace fbl {

const double akh = 0.5; // fixed angle between the bar and the line connecting hip-knee joints
const double fkb = 0.6; // fixed angle between the bar and the line connecting knee and foo

// The linkage bars:
const double ak = 0.040;
const double ar = 0.070;
const double bk = 0.030;
const double br = 0.070;
const double cj = 0.035;
const double jk = 0.360 + 0.040;

const double ak2 = ak*ak;
const double bk2 = bk*bk;
const double ar2 = ar*ar;
const double br2 = br*br;

// Coming from Marco Frigerio's previous work
/**
 * Four-bar linkage mechanical constants
 */
/*const iit::commons::FBLConsts fblConsts = {
    akh : 0.5,
    cj  : 0.035,
    jk  : 0.360 + 0.040,

    // The linkage bars:
    ak : 0.04,
    bk : 0.03,
    ar : 0.07,
    br : 0.07
};*/

const double haa_x = 0.2399; // Updated to Aliengo: <xacro:property name="leg_offset_x" value="0.2399"/>
const double haa_y = 0.051; // Updated to Aliengo: <xacro:property name="leg_offset_y" value="0.051"/>

/** Left-Right distance between two HAA axes*/
const double dLR = haa_x * 2;
/** Front-Hind distance between the origins of two HAA frames*/
const double dFH = haa_y * 2;
// End of coming from Marco Frigerio's previous work

/**
 * Total length of the cylinder (between attachment points) when the rod is
 * fully retracted.
 *
 * Note that it is irrelevant whether the robot can or cannot actually reach the
 * configuration where the cylinder is fully retracted.
 *
 * This value can be used to compute how much the rod is extended given the
 * current total length of the cylinder assembly, which in turn has to be
 * computed with the geometry of the four bar linkage.
 */
const double cyl_retracted = 0.3131076;

/**
 * Distance between ISA rod ends (normally measured in CAD) when the leg assumes
 * the posture imposed by the calibration frame.
 *
 * The value showed below correspond to the calibration frame that imposes the
 * following joint angles for all the legs: 15 degress for HAA, 0 degree for HFE
 * and -100 degrees for KFE.
 */
const double actuator_rod_ends_dist_at_cal_position = 0.3464;

// The triangle CJK; it is invariant regardless the linkage status
const double ck2 = cj*cj + jk*jk;
const double ck = std::sqrt(ck2);
const double ckh = std::acos(jk/ck);

/**
 * The constant angle between the bar which is fixed in the lower leg
 * (segment KB in our model) and the X axis of the lower leg reference
 * frame.
 */
const double angle_KB_lleg_xaxis = 0.6;

}

/**
 * The joint angle (in radians) that corresponds to an actuator position state
 * equal to 0, according to the actuator conventions (ie no oil in the positive
 * force chamber).
 * In other words, this value should be the LF_HFE status when touching the
 * internal actuator end-stop, with the leg being raised "forward" (which is
 * the most negative joint angle).
 * This magic constant depends on the actual assembly of the robot.
*/
const double LF_HFE_JOINT_MOTOR_OFF = -2.356; // -135 [°] is -2.356 [rad]
/**
 * See LF_HFE_JOINT_MOTOR_OFF
 */
const double LF_HAA_JOINT_MOTOR_OFF = -0.698; // -40 [°] is -0.698 [rad]

}
}




#endif
