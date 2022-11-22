/*!
 * @file utils.hpp
 *
 * @brief Utils file with functions prototypes
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ALIENGOLIB_UTILS_HPP_
#define _ALIENGOLIB_UTILS_HPP_

#include <cmath>
#include "robcogen/rbd.h"

#include "dog/leg_data_map.h"
#include "dog/declarations.h"
#include "dog/inertia_properties.h"
#include "dog/inverse_dynamics.h"

#include "geometry/rotations.h"

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

    //! Path to the aliengo urdf.
    inline const std::string aliengo_urdf_path {"/usr/lib/robots/aliengo.urdf"};

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

namespace iit
{
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
    // ******************** ___FROM iit/commons/rbd/utils.h___ ********************
    
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
    /*!
     * @brief Return the number of legs that are in stance phase
     * 
     * @param[in] stance_legs Boolean values associated to each leg, that indicate if these are in stance or not
     * @return int
     */
    int compute_stance_legs(const iit::dog::LegDataMap<bool> & stance_legs);
    /*!
     * @brief Get the robot center of mass (CoM) with respect to the base
     * 
     * @param[in] q robot joints state
     * @param[in] base_orient robot base orientation expressed in the world frame
     * @param[in] base_pos robot base position expressed in the world frame
     * @param[in] in parameter used to get the COM offset expressed in the base frame
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d getCoMFromBase(const iit::dog::JointState & q,
                                   const Eigen::Vector3d & base_orient,
                                   const Eigen::Vector3d & base_pos,
                                   iit::dog::InertiaPropertiesBase& in);
    /*!
     * @brief Get the robot base with respect to the center of mass (CoM)
     * 
     * @param[in] q robot joints state
     * @param[in] base_orient robot base orientation expressed in the world frame
     * @param[in] CoM robot center of mass expressed in the world frame
     * @param[in] in parameter used to get the COM offset expressed in the base frame
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d getBaseFromCoM(const iit::dog::JointState & q,
                                   const Eigen::Vector3d & base_orient,
                                   const Eigen::Vector3d & CoM,
                                   iit::dog::InertiaPropertiesBase &in);

    // ******************** ___FROM dls_commons/liblocomotionutils/computeJacobians.cpp___ ********************
} // namespace iit

#endif // _ALIENGO_UTILS_HPP_