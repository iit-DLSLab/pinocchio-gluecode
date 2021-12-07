#include "utils.hpp"
#include "pugixml/pugixml.hpp"

#include <iostream>

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

	void evaluate_forward_kinematics(struct FK_In* inputs, struct KinematicsConfig* config, struct FK_Out* outputs)
    {
        double dh_d[4], dh_a[4], dh_ca[4], dh_sa[4], dh_do[4];
        double bo =  config->base_offset;
        double th_v = config->thigh_vertical;
        double th_h = config->thigh_horizontal;
        double ko = config->knee_offset;
        double th = config->thigh;
        double sh = config->shank;
        double *ja_fk_in = &(inputs->cur_joints[0]);

        unsigned short i = 0;
        double dh_t, st, ct, sa, ca;
        double T_Fr_2_Fr[16], T_Base_2_TCP_tmp[16], T_Base_2_TCP_fk_out[16];

        /* define the Denavit-Hartenberg parameters */
        dh_d[0] = bo;   dh_a[0] = 0.0;  dh_ca[0] = 0.0; dh_sa[0] = 1.0; dh_do[0] = 0.0;
        dh_d[1] = th_v; dh_a[1] = 0.0;  dh_ca[1] = 0.0; dh_sa[1] =-1.0; dh_do[1] = 0.0;
        dh_d[2] = th_h; dh_a[2] = th;   dh_ca[2] = 1.0; dh_sa[2] = 0.0; dh_do[2] = M_PI_2;
        dh_d[3] = ko;   dh_a[3] = sh;   dh_ca[3] = 1.0; dh_sa[3] = 0.0; dh_do[3] = M_PI;

        for (i = 0; i < 4; ++i) {

            /* Adaption to the joint delta angle offsets */
            dh_t = ja_fk_in[i] + dh_do[i];

            /* calc sin and cos just once */
            st = sin(dh_t);
            ct = cos(dh_t);
            sa = dh_sa[i];
            ca = dh_ca[i];

            /* build the transformation matrix from frame to frame */
            T_Fr_2_Fr[0] =  ct;    T_Fr_2_Fr[4] = -st * ca;   T_Fr_2_Fr[8]  =  st * sa;   T_Fr_2_Fr[12] = ct * dh_a[i];
            T_Fr_2_Fr[1] =  st;    T_Fr_2_Fr[5] =  ct * ca;   T_Fr_2_Fr[9]  = -ct * sa;   T_Fr_2_Fr[13] = st * dh_a[i];
            T_Fr_2_Fr[2] = 0.0;    T_Fr_2_Fr[6] =     sa;   T_Fr_2_Fr[10] =     ca;   T_Fr_2_Fr[14] =    dh_d[i];
            T_Fr_2_Fr[3] = 0.0;    T_Fr_2_Fr[7] =    0.0;   T_Fr_2_Fr[11] =    0.0;   T_Fr_2_Fr[15] =        1.0;

            /* multiply iteratively the transformation matrix from frame to frame with the transformation matrix to the base */
            if (i == 0) {
                copy_mat_4x4(T_Fr_2_Fr, T_Base_2_TCP_fk_out);
            } else {
                copy_mat_4x4(T_Base_2_TCP_fk_out, T_Base_2_TCP_tmp);
                mult_mat_mat_4x4(T_Base_2_TCP_tmp, T_Fr_2_Fr, T_Base_2_TCP_fk_out);
            }
        }

        outputs->pos_x = T_Base_2_TCP_fk_out[12];
        outputs->pos_y = T_Base_2_TCP_fk_out[13];
        outputs->pos_z = T_Base_2_TCP_fk_out[14];

        outputs->rot_w = sqrt(1.0 + T_Base_2_TCP_fk_out[0] + T_Base_2_TCP_fk_out[5] + T_Base_2_TCP_fk_out[10]) / 2.0;
        outputs->rot_x = (T_Base_2_TCP_fk_out[6] - T_Base_2_TCP_fk_out[9]) / (4.0 * outputs->rot_w);
        outputs->rot_y = (T_Base_2_TCP_fk_out[8] - T_Base_2_TCP_fk_out[2]) / (4.0 * outputs->rot_w);
        outputs->rot_z = (T_Base_2_TCP_fk_out[1] - T_Base_2_TCP_fk_out[4]) / (4.0 * outputs->rot_w);
    }

    void evaluate_inverse_kinematics(struct IK_In* inputs, struct KinematicsConfig* config, struct IK_Out* outputs)
    {
        double x, y, z;
        double sin_lean, cos_lean;
        double th_kn_o, s_sq, k_sq, k, d_sq, d, th, sh, dist, max_dist;
        double alpha, beta, gamma, delta;
        unsigned short dof_nr, NR_DOF_LEG;
        double* j_ik_out = &(outputs->des_joints[0]);
        double* j_last = &(inputs->last_joints[0]);
        double* j_max = &(config->max_joints[0]);
        double* j_min = &(config->min_joints[0]);
        NR_DOF_LEG = 4;

        /* the angle of the first joint is the lean angle */
        j_ik_out[0] = j_last[0];

        /* go to the coordinate system of the thorax joint (where x points towards tcp and z towards vertical z axis) */
        sin_lean = sin(j_last[0]);
        cos_lean = cos(j_last[0]);
        x = inputs->pos_z - config->base_offset;
        y = inputs->pos_x * cos_lean + inputs->pos_y * sin_lean;
        z = inputs->pos_y * cos_lean - inputs->pos_x * sin_lean;

        /* calculate some help variables for the thorax joint */
        /*printf("[IK] x %f, y %f, z %f\n", x, y, z);*/
        th_kn_o = config->thigh_horizontal + config->knee_offset;
        s_sq = x * x + y * y;
        k_sq = s_sq - th_kn_o * th_kn_o;

        if (k_sq > 0) {
            k = sqrt(k_sq);
        } else {
            k = 0.0;
        }
        alpha = atan2(k, th_kn_o);

        beta = atan2(y, x);
        /*printf("k %f, alpha %f, beta %f\n", k, alpha*RAD2DEG, beta*RAD2DEG);*/

        /* calculate the thorax angle (depending on thorax configuration)*/
        j_ik_out[1] = -config->thorax_conf * alpha - beta;

        /* calculate some help variables for the basal and distal joint */
        z += config->thigh_vertical;
        d_sq = k * k + z * z;
        d = sqrt(d_sq);


        /* quit if the position is not reachable and take the last valid angles */
        th = config->thigh;
        sh = config->shank;

        max_dist = th + sh;
        if (max_dist < d) {
            for (dof_nr = 0; dof_nr < NR_DOF_LEG; dof_nr++) {
                j_ik_out[dof_nr] = j_last[dof_nr];
            }
            outputs->min_dist = max_dist - d;
            outputs->min_dist_dof = 99;                   /* pos too far away */
            return;
        }

        /*printf("d %f, th %f, sh %f\n", d, th, sh);*/

        if (fabs(th - sh) > d) {
            for (dof_nr = 0; dof_nr < NR_DOF_LEG; dof_nr++) {
                j_ik_out[dof_nr] = j_last[dof_nr];
            }
            outputs->min_dist = -fabs(th - sh) + d;
            outputs->min_dist_dof = 98;                   /* pos to close too the shoulder */
            return;
        }

        /* calculate some help angles */
        if (d_sq == 0) {
            return;
        } else {
            gamma = acos( (th * th + d_sq - sh * sh) / (2.0 * th * d) );
        }
        delta = atan2(z, k);

        /* calculate the last joint angles */
        j_ik_out[2] =  M_PI / 2.0 - gamma - delta;
        j_ik_out[3] = -acos( (th * th + sh * sh - d_sq) / (2.0 * th * sh) );

        /* printf("z %f, d %f, gamma %f, delta %f\n", z, d, gamma*RAD2DEG, delta*RAD2DEG); */

        j_ik_out[2] *= config->thorax_conf;
        j_ik_out[3] *= config->thorax_conf;

        /* Remark: a second solution is to have the knee below the end effector */
        /*         it is not taken into consideration, because this solution is likely to hit the ground */

        outputs->min_dist = 2.0 * M_PI;
        for (dof_nr = 0; dof_nr < NR_DOF_LEG; dof_nr++) {
            j_ik_out[dof_nr] = bound(j_ik_out[dof_nr], j_min[dof_nr], j_max[dof_nr]);
            j_last[dof_nr] = j_ik_out[dof_nr];

            /* give information which joint is closed to a border */
            dist = fmin(j_max[dof_nr] - j_ik_out[dof_nr], j_ik_out[dof_nr] - j_min[dof_nr]);
            if (dist < outputs->min_dist) {
                outputs->min_dist = dist;
                outputs->min_dist_dof = dof_nr;
            }
            if (dist < 0.0) {
                j_ik_out[dof_nr] = j_last[dof_nr];
            }
        }

        /* it is also critical when the elbow is almost straight */
        dist = fabs(j_ik_out[3]);
        if (dist < outputs->min_dist) {
            outputs->min_dist = dist;
            outputs->min_dist_dof = 3;
        }
    }

    const std::string readURDFPugixml(const std::string& urdf_path)
    {
        // Create empty XML document within memory
        pugi::xml_document doc;
        // Load XML file into memory
        // Remark: to fully read declaration entries you have to specify
        // "pugi::parse_declaration"
        pugi::xml_parse_result result = doc.load_file(urdf_path.c_str());
        if (!result){
            std::cout << "error while loading the aliengo urdf: " << result.description() << std::endl; 
        }
        std::stringstream ss;
        doc.save(ss," ");

        return ss.str();        
    }
} // namespace aliengolib


namespace iit{
 
    static Eigen::Matrix3d buildCrossProductMatrix(const Eigen::Vector3d& in) {
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