/**
 * @file utils.cpp
 *
 * @brief Utils file with functions implementation
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

#include "utils.hpp"

#include <iostream>
#include <fstream>

namespace aliengolib
{
	void copy_mat_4x4(double src[16], double dest[16]) {
		unsigned short j;
		for (j = 0; j < 16; j++)
			dest[j] = src[j];
	}

	void mult_mat_mat_4x4(double src1[16], double src2[16], double dest[16])
	{
		dest[0]  = src1[0] * src2[0] + src1[4] * src2[1] + src1[8] * src2[2] + src1[12] * src2[3];
		dest[1]  = src1[1] * src2[0] + src1[5] * src2[1] + src1[9] * src2[2] + src1[13] * src2[3];
		dest[2]  = src1[2] * src2[0] + src1[6] * src2[1] + src1[10] * src2[2] + src1[14] * src2[3];
		dest[3]  = src1[3] * src2[0] + src1[7] * src2[1] + src1[11] * src2[2] + src1[15] * src2[3];
		dest[4]  = src1[0] * src2[4] + src1[4] * src2[5] + src1[8] * src2[6] + src1[12] * src2[7];
		dest[5]  = src1[1] * src2[4] + src1[5] * src2[5] + src1[9] * src2[6] + src1[13] * src2[7];
		dest[6]  = src1[2] * src2[4] + src1[6] * src2[5] + src1[10] * src2[6] + src1[14] * src2[7];
		dest[7]  = src1[3] * src2[4] + src1[7] * src2[5] + src1[11] * src2[6] + src1[15] * src2[7];
		dest[8]  = src1[0] * src2[8] + src1[4] * src2[9] + src1[8] * src2[10] + src1[12] * src2[11];
		dest[9]  = src1[1] * src2[8] + src1[5] * src2[9] + src1[9] * src2[10] + src1[13] * src2[11];
		dest[10] = src1[2] * src2[8] + src1[6] * src2[9] + src1[10] * src2[10] + src1[14] * src2[11];
		dest[11] = src1[3] * src2[8] + src1[7] * src2[9] + src1[11] * src2[10] + src1[15] * src2[11];
		dest[12] = src1[0] * src2[12] + src1[4] * src2[13] + src1[8] * src2[14] + src1[12] * src2[15];
		dest[13] = src1[1] * src2[12] + src1[5] * src2[13] + src1[9] * src2[14] + src1[13] * src2[15];
		dest[14] = src1[2] * src2[12] + src1[6] * src2[13] + src1[10] * src2[14] + src1[14] * src2[15];
		dest[15] = src1[3] * src2[12] + src1[7] * src2[13] + src1[11] * src2[14] + src1[15] * src2[15];
	}

	double bound(double val, double min, double max) { return fmin(fmax(val, min), max); }
    
    const std::string readURDFifstream(const std::string& urdf_path)
    {
        std::ifstream myfile{urdf_path};
        std::stringstream ss{};
        if (myfile.is_open())
        {
            ss << myfile.rdbuf();
        }
        else
        {
            std::cout << "Failed to read the urdf using ifstream\n" << std::endl;
            std::cout << "Maybe you don't have the urdf file located in the project folder. Please make sure that the file " << aliengo_urdf_path << " exists." << std::endl;
            ss << "";
        }
        
        return ss.str();
    }
} // namespace aliengolib

namespace iit{
 
    Eigen::Matrix3d buildCrossProductMatrix(const Eigen::Vector3d& in) {
        Eigen::Matrix3d out;
        out <<  0   , -in(2),  in(1),
               in(2),   0   , -in(0),
              -in(1),  in(0),   0;
        return out;
    }
    
    iit::rbd::Matrix66d motionVectorTransform(const iit::rbd::Vector3d & position, const Eigen::Matrix3d & rotationMx)
    {
        iit::rbd::Matrix66d X = iit::rbd::Matrix66d::Zero();

        X.block<3,3>(iit::rbd::AX, iit::rbd::AX) = rotationMx;
        X.block<3,3>(iit::rbd::LX, iit::rbd::AX) = -rotationMx*buildCrossProductMatrix(position);
        X.block<3,3>(iit::rbd::LX, iit::rbd::LX) = rotationMx;

        return X;
    }

    iit::rbd::Matrix66d forceVectorTransform(const iit::rbd::Vector3d & position, const Eigen::Matrix3d & rotationMx)
    {
        iit::rbd::Matrix66d X = iit::rbd::Matrix66d::Zero();

        X.block<3,3>(iit::rbd::AX, iit::rbd::AX) = rotationMx;
        X.block<3,3>(iit::rbd::AX, iit::rbd::LX) = -rotationMx*buildCrossProductMatrix(position);
        X.block<3,3>(iit::rbd::LX, iit::rbd::LX) = rotationMx;

        return X;
    }

    int compute_stance_legs(const iit::dog::LegDataMap<bool> & stance_legs)
    {
        int cleg_count = 0;
        for (int i = 0; i<iit::dog::_LEGS_COUNT; i++){
            if (stance_legs[iit::dog::LegID(i)])
                cleg_count++;
        }
        return cleg_count;
    }

    Eigen::Vector3d getCoMFromBase(const iit::dog::JointState & q,
                                   const Eigen::Vector3d & base_orient,
                                   const Eigen::Vector3d & base_pos,
                                   iit::dog::InertiaPropertiesBase& in)
    {
        Eigen::Matrix3d R = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = in.getWholeBodyCOM(q);
        return base_pos + R.transpose()*offCoM; //base_pos is in the world frame. offCoM is in base frame
    }

    Eigen::Vector3d getBaseFromCoM(const iit::dog::JointState & q,
                                   const Eigen::Vector3d & base_orient,
                                   const Eigen::Vector3d & CoM,
                                   iit::dog::InertiaPropertiesBase &in)
    {
        Eigen::Matrix3d b_R_w = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = in.getWholeBodyCOM(q);
        return CoM - b_R_w.transpose()*offCoM; //CoM is in the world frame. offCoM is in base frame
    }
} // namespace iit