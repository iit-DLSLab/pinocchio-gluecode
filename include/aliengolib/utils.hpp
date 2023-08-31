#ifndef IIT_ALIENGOLIB_UTILS_HPP_
#define IIT_ALIENGOLIB_UTILS_HPP_

#include <cmath>
#include "aliengolib/robcogen/rbd.h"

#include "aliengolib/dog/leg_data_map.h"
#include "aliengolib/dog/declarations.h"
#include "aliengolib/dog/transforms.h"
#include "aliengolib/dog/inertia_properties.h"
#include "aliengolib/dog/inverse_dynamics.h"

#include "aliengolib/geometry/rotations.h"

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

    inline std::map<std::string, iit::dog::DestFrame> glue_destination_frame_names_to_ids
    {
        {"TRUNK", iit::dog::DestFrame::TRUNK},
        {"LF_LOWERLEG", iit::dog::DestFrame::LF_LOWERLEG},
        {"RF_LOWERLEG", iit::dog::DestFrame::RF_LOWERLEG},
        {"LH_LOWERLEG", iit::dog::DestFrame::LH_LOWERLEG},
        {"RH_LOWERLEG", iit::dog::DestFrame::RH_LOWERLEG},
        {"LF_FOOT", iit::dog::DestFrame::LF_FOOT},
        {"RF_FOOT", iit::dog::DestFrame::RF_FOOT},
        {"LH_FOOT", iit::dog::DestFrame::LH_FOOT},
        {"RH_FOOT", iit::dog::DestFrame::RH_FOOT},
        {"LF_HIPASSEMBLY", iit::dog::DestFrame::LF_HIPASSEMBLY},
        {"RF_HIPASSEMBLY", iit::dog::DestFrame::RF_HIPASSEMBLY},
        {"LH_HIPASSEMBLY", iit::dog::DestFrame::LH_HIPASSEMBLY},
        {"RH_HIPASSEMBLY", iit::dog::DestFrame::RH_HIPASSEMBLY},
        {"LF_UPPERLEG", iit::dog::DestFrame::LF_UPPERLEG},
        {"RF_UPPERLEG", iit::dog::DestFrame::RF_UPPERLEG},
        {"LH_UPPERLEG", iit::dog::DestFrame::LH_UPPERLEG},
        {"RH_UPPERLEG", iit::dog::DestFrame::RH_UPPERLEG},
    };

    inline std::map<std::string, iit::dog::OriginFrame> glue_origin_frame_names_to_ids
    {
        {"TRUNK", iit::dog::OriginFrame::TRUNK},
        {"LF_HAA", iit::dog::OriginFrame::LF_HAA},
        {"LF_HFE", iit::dog::OriginFrame::LF_HFE},
        {"LF_KFE", iit::dog::OriginFrame::LF_KFE},
        {"RF_HAA", iit::dog::OriginFrame::RF_HAA},
        {"RF_HFE", iit::dog::OriginFrame::RF_HFE},
        {"RF_KFE", iit::dog::OriginFrame::RF_KFE},
        {"LH_HAA", iit::dog::OriginFrame::LH_HAA},
        {"LH_HFE", iit::dog::OriginFrame::LH_HFE},
        {"LH_KFE", iit::dog::OriginFrame::LH_KFE},
        {"RH_HAA", iit::dog::OriginFrame::RH_HAA},
        {"RH_HFE", iit::dog::OriginFrame::RH_HFE},
        {"RH_KFE", iit::dog::OriginFrame::RH_KFE},
        {"LF_LOWERLEG", iit::dog::OriginFrame::LF_LOWERLEG},
        {"RF_LOWERLEG", iit::dog::OriginFrame::RF_LOWERLEG},
        {"LH_LOWERLEG", iit::dog::OriginFrame::LH_LOWERLEG},
        {"RH_LOWERLEG", iit::dog::OriginFrame::RH_LOWERLEG},
        {"LF_HIPASSEMBLY_COM", iit::dog::OriginFrame::LF_HIPASSEMBLY_COM},
        {"RF_HIPASSEMBLY_COM", iit::dog::OriginFrame::RF_HIPASSEMBLY_COM},
        {"LH_HIPASSEMBLY_COM", iit::dog::OriginFrame::LH_HIPASSEMBLY_COM},
        {"RH_HIPASSEMBLY_COM", iit::dog::OriginFrame::RH_HIPASSEMBLY_COM},
        {"LF_HIPASSEMBLY", iit::dog::OriginFrame::LF_HIPASSEMBLY},
        {"RF_HIPASSEMBLY", iit::dog::OriginFrame::RF_HIPASSEMBLY},
        {"LH_HIPASSEMBLY", iit::dog::OriginFrame::LH_HIPASSEMBLY},
        {"RH_HIPASSEMBLY", iit::dog::OriginFrame::RH_HIPASSEMBLY},
        {"LF_UPPERLEG_COM", iit::dog::OriginFrame::LF_UPPERLEG_COM},
        {"RF_UPPERLEG_COM", iit::dog::OriginFrame::RF_UPPERLEG_COM},
        {"LH_UPPERLEG_COM", iit::dog::OriginFrame::LH_UPPERLEG_COM},
        {"RH_UPPERLEG_COM", iit::dog::OriginFrame::RH_UPPERLEG_COM},
        {"LF_UPPERLEG", iit::dog::OriginFrame::LF_UPPERLEG},
        {"RF_UPPERLEG", iit::dog::OriginFrame::RF_UPPERLEG},
        {"LH_UPPERLEG", iit::dog::OriginFrame::LH_UPPERLEG},
        {"RH_UPPERLEG", iit::dog::OriginFrame::RH_UPPERLEG},
        {"LF_LOWERLEG_COM", iit::dog::OriginFrame::LF_LOWERLEG_COM},
        {"RF_LOWERLEG_COM", iit::dog::OriginFrame::RF_LOWERLEG_COM},
        {"LH_LOWERLEG_COM", iit::dog::OriginFrame::LH_LOWERLEG_COM},
        {"RH_LOWERLEG_COM", iit::dog::OriginFrame::RH_LOWERLEG_COM},
        {"LF_SHIN", iit::dog::OriginFrame::LF_SHIN},
        {"RF_SHIN", iit::dog::OriginFrame::RF_SHIN},
        {"LH_SHIN", iit::dog::OriginFrame::LH_SHIN},
        {"RH_SHIN", iit::dog::OriginFrame::RH_SHIN},
        {"LF_FOOT", iit::dog::OriginFrame::LF_FOOT},
        {"RF_FOOT", iit::dog::OriginFrame::RF_FOOT},
        {"LH_FOOT", iit::dog::OriginFrame::LH_FOOT},
        {"RH_FOOT", iit::dog::OriginFrame::RH_FOOT},
    };

    void copy_mat_4x4(double src[16], double dest[16]);
    void mult_mat_mat_4x4(double src1[16], double src2[16], double dest[16]);
    double bound(double val, double min, double max);
    
    const std::string readURDFifstream(const std::string& urdf_path);
        
} //namespace aliengolib

namespace iit{

    // ******************** FROM iit/commons/rbd/utils.h ********************    
    Eigen::Matrix3d buildCrossProductMatrix(const Eigen::Vector3d& in);
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

#endif // _ALIENGO_UTILS_HPP_
