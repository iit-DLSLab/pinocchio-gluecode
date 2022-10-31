#include "aliengolib/utils.hpp"

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
            std::string robot_description{ss.str()};
        }
        else
        {
            std::cout << "Failed to read the urdf using ifstream\n" << std::endl;
            std::cout << "Maybe you don't have the urdf file located in the project folder. So you have to:" << std::endl;
            std::cout << "- generate the urdf from the build folder ->  . ../generate_urdf.txt <path_to_xacro_file> (e.g. . ../generate_urdf.txt $HOME/$ROS_WORKSPACE_NAME/src/dls-distro/robots/aliengo/description/robots/aliengo.urdf.xacro)\n" 
                      << "    You can skip this passage if you already have in aliengolib the urdf version you want" << std::endl;
            std::cout << "- launch the tests from within the build folder" << std::endl;
            
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
    
    /**
     * @brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
     * where A is the origin frame and B the destination frame.
     * @param position coordinate vector expressing OaOb in A coordinates
     * @param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return
     */
    iit::rbd::Matrix66d motionVectorTransform(const iit::rbd::Vector3d & position,
                                        const Eigen::Matrix3d & rotationMx)
    {
        iit::rbd::Matrix66d X=iit::rbd::Matrix66d::Zero();

        X.block<3,3>(iit::rbd::AX, iit::rbd::AX) = rotationMx;
        X.block<3,3>(iit::rbd::LX, iit::rbd::AX) = -rotationMx*buildCrossProductMatrix(position);
        X.block<3,3>(iit::rbd::LX, iit::rbd::LX) = rotationMx;

        return X;
    }

    /**
     * @brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
     * where A is the origin frame and B the destination frame.
     * @param position coordinate vector expressing OaOb in A coordinates
     * @param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
     * @return
     */
    iit::rbd::Matrix66d forceVectorTransform(const iit::rbd::Vector3d & position,
                                        const Eigen::Matrix3d & rotationMx)
    {
        iit::rbd::Matrix66d X=iit::rbd::Matrix66d::Zero();

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
                                iit::dog::InertiaPropertiesBase& in)
    {
        Eigen::Matrix3d R = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = in.getWholeBodyCOM(q);
        return base_pos + R.transpose()*offCoM; //CoM is in the world frame off CoM is in base frame
    }

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
                                iit::dog::InertiaPropertiesBase &in)
    {
            Eigen::Matrix3d b_R_w = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = in.getWholeBodyCOM(q);
            return CoM - b_R_w.transpose()*offCoM; //CoM is in the world frame off CoM is in base frame
    }
}