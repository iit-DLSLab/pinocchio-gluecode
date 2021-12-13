#ifndef _CREXLIB_UTILS_HPP_
#define _CREXLIB_UTILS_HPP_

#include <cmath>
#include "types.hpp"
#include "robcogen/rbd.h"

#include "dog/leg_data_map.h"
#include "dog/declarations.h"
#include "dog/inertia_properties.h"
#include "dog/inverse_dynamics.h"

#include "geometry/rotations.h"

#include <sstream>

namespace aliengolib
{
    // inline is used to avoid the multiple definition error at linking time
    inline std::map<std::string, int> glue_joint_names_to_ids
    {
        {"LF_HAA", 0},
        {"LF_HFE", 1},
        {"LF_KFE", 2},
        {"RF_HAA", 3},
        {"RF_HFE", 4},
        {"RF_KFE", 5},
        {"LH_HAA", 6},
        {"LH_HFE", 7},
        {"LH_KFE", 8},
        {"RH_HAA", 9},
        {"RH_HFE", 10},
        {"RH_KFE", 11}
    };

    // inline is used to avoid the multiple definition error at linking time
    inline std::map<std::string, int> glue_leg_names_to_ids
    {
        {"LF", 0},
        {"RF", 1},
        {"LH", 2},
        {"RH", 3},
    };

    // dfki_fk **********
    void copy_mat_4x4(double src[16], double dest[16]);
    void mult_mat_mat_4x4(double src1[16], double src2[16], double dest[16]);
    double bound(double val, double min, double max);
    // TODO: inheritance from robotBase?
    void evaluate_forward_kinematics(struct FK_In* inputs, struct KinematicsConfig* config, struct FK_Out* outputs);
    void evaluate_inverse_kinematics(struct IK_In* inputs, struct KinematicsConfig* config, struct IK_Out* outputs);

    const std::string readURDFifstream(const std::string& urdf_path);
        
} //namespace aliengolib

namespace iit{

    // ******************** FROM iit/commons/rbd/utils.h ********************    
    static Eigen::Matrix3d buildCrossProductMatrix(const Eigen::Vector3d& in);
    // ******************** ___FROM iit/commons/rbd/utils.h___ ********************
    

    // ******************** FROM dls_commons/liblocomotionutils/computeJacobians.cpp ********************
    /**
     * @brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
     * where A is the origin frame and B the destination frame.
     * @param position coordinate vector expressing OaOb in A coordinates
     * @param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return
     */
    iit::rbd::Matrix66d motionVectorTransform(const iit::rbd::Vector3d & position,
                                        const Eigen::Matrix3d & rotationMx);

    /**
     * @brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
     * where A is the origin frame and B the destination frame.
     * @param position coordinate vector expressing OaOb in A coordinates
     * @param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return
     */
    iit::rbd::Matrix66d forceVectorTransform(const iit::rbd::Vector3d & position,
                                        const Eigen::Matrix3d & rotationMx);

    int compute_stance_legs(const iit::dog::LegDataMap<bool> & stance_legs);

    /**
     * @brief getCoMFromBase
     * @param q
     * @param base_orient
     * @param base_pos  base is supposed to be expressed in the world frame
     * @param in
     * @return
     */
    Eigen::Vector3d getCoMFromBase(const iit::dog::JointState & q,
                                const Eigen::Vector3d & base_orient,
                                const Eigen::Vector3d & base_pos,
                                iit::dog::InertiaPropertiesBase& in);

    /**
     * @brief getBaseFromCoM
     * @param q
     * @param base_orient
     * @param CoM CoM position in world coordinates
     * @param in
     * @return
     */
    Eigen::Vector3d getBaseFromCoM(const iit::dog::JointState & q,
                                const Eigen::Vector3d & base_orient,
                                const Eigen::Vector3d & CoM,
                                iit::dog::InertiaPropertiesBase &in);

    // ******************** ___FROM dls_commons/liblocomotionutils/computeJacobians.cpp___ ********************

} // namespace iit

#endif // _CREXLIB_UTILS_HPP_
