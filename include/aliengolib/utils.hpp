/*!
 * @file utils.hpp
 *
 * @brief Utils file with functions prototypes
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ALIENGOLIB_UTILS_HPP_
#define _ALIENGOLIB_UTILS_HPP_

#include <cmath>
#include "robcogen/rbd.h"

#include "aliengolib/dog/leg_data_map.h"
#include "aliengolib/dog/declarations.h"
#include "aliengolib/dog/inertia_properties.h"
#include "aliengolib/dog/inverse_dynamics.h"
#include "aliengolib/dog/transforms.h"

#include "aliengolib/geometry/rotations.h"

#include <sstream>

namespace aliengolib
{
	/*!
     * @brief Utils functions.
     * @details
     * This file contains a list of useful functions that are user all over the library.
     */

    //! Mapping the names of the joints to ids.
    inline const std::map<std::string, int> glue_joint_names_to_ids
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

    //! Mapping the names of the legs to ids.
    inline const std::map<std::string, int> glue_leg_names_to_ids
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

    //! Path to the aliengo urdf.
    inline const std::string aliengo_urdf_path {"/usr/include/aliengo_description/urdfs/aliengo.urdf"};

    /*!
    * @brief Copy the source 4x4 matrix into the destination one.
    *
    * @param[in] src 4x4 matrix (decomposed row-by-row to array) from which values are copied.
    * @param[out] dest 4x4 matrix (decomposed row-by-row to array) to which values are copied.
    */
    void copy_mat_4x4(double src[16], double dest[16]);
    /*!
    * @brief Multiply (row * column) the two 4x4 matrices and save the result in the destination matrix.
    *
    * @param[in] src1 4x4 matrix (decomposed row-by-row to array) multiplied by column.
    * @param[in] src2 4x4 matrix (decomposed row-by-row to array) multiplied by row.
    * @param[out] dest 4x4 matrix (decomposed row-by-row to array) to which values are copied.
    */
    void mult_mat_mat_4x4(double src1[16], double src2[16], double dest[16]);
    /*!
    * @brief Return the value between the minimum/maximum values. If the value is lower/higher than the minimum/maximum one, return the minimium/maximum value.
    *
    * @param[in] val value to be checked.
    * @param[in] min lower bound for the value to be checked.
    * @param[in] max higher bound for the value to be checked.
    * @return value between the minimum/maximum values.
    */
    double bound(double val, double min, double max);
    /*!
    * @brief Return the robot URDF in the form of a string.
    *
    * @param[in] urdf_path the path of the robot URDF.
    * @return urdf in string format.
    */
    const std::string readURDFifstream(const std::string& urdf_path);
        
} //namespace aliengolib

namespace iit{

     // ******************** FROM iit/commons/rbd/utils.h ********************
    /*!
     * @brief Return the skew-symmetric matrix from the vector in input.
     * The output can be multiplied with a second vector (row*column multiplication) to obtain the cross product 
     * between the first vector in input and the second vector  
     * 
     * @param[in] in vector used to create a skew-symmetric matrix
     * @return Eigen::Matrix3d
     */
    Eigen::Matrix3d buildCrossProductMatrix(const Eigen::Vector3d& in);
    
    // ******************** FROM dls_commons/liblocomotionutils/computeJacobians.cpp ********************
    /*!
     * @brief Tranform twists from A to B (b_X_a) (R^6*6), where A is the origin frame and B the destination frame
     * 
     * @param[in] position coordinate vector expressing OaOb in A coordinates
     * @param[in] rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return Eigen::Matrix<double, 6, 6>
     */
    iit::rbd::Matrix66d motionVectorTransform(const iit::rbd::Vector3d & position, const Eigen::Matrix3d & rotationMx);
    /*!
     * @brief Tranform wrenches from A to B (b_X_a) (R^6*6), where A is the origin frame and B the destination frame
     * 
     * @param[in] position coordinate vector expressing OaOb in A coordinates
     * @param[in] rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return Eigen::Matrix<double, 6, 6>
     */
    iit::rbd::Matrix66d forceVectorTransform(const iit::rbd::Vector3d & position, const Eigen::Matrix3d & rotationMx);

    // ******************** ___FROM dls_commons/liblocomotionutils/computeJacobians.cpp___ ********************

} // namespace iit

#endif // _ALIENGO_UTILS_HPP_