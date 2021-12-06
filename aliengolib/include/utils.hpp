#ifndef _CREXLIB_UTILS_HPP_
#define _CREXLIB_UTILS_HPP_

#include <cmath>
#include "types.hpp"

namespace aliengolib
{
    // dfki_fk **********
    void copy_mat_4x4(double src[16], double dest[16]);
    void mult_mat_mat_4x4(double src1[16], double src2[16], double dest[16]);
    double bound(double val, double min, double max);
    // TODO: inheritance from robotBase?
    void evaluate_forward_kinematics(struct FK_In* inputs, struct KinematicsConfig* config, struct FK_Out* outputs);
    void evaluate_inverse_kinematics(struct IK_In* inputs, struct KinematicsConfig* config, struct IK_Out* outputs);
} //namespace aliengolib


#endif // _CREXLIB_UTILS_HPP_
