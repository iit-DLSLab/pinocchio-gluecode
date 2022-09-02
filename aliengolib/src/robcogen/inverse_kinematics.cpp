#include "robcogen/inverse_kinematics.h"

using namespace iit::dog;

namespace iit {
namespace Aliengo {

InverseKinematics::InverseKinematics(const dog::KinDynParams &default_pg) :
    default_pg_(default_pg),
    jacobians_(default_pg),
    q_min_(JointState::Constant(-10.0)),
    q_max_(JointState::Constant(10.0)) {

    feet_jacobians_[LF] = &jacobians_.fr_trunk_J_LF_foot;
    feet_jacobians_[RF] = &jacobians_.fr_trunk_J_RF_foot;
    feet_jacobians_[LH] = &jacobians_.fr_trunk_J_LH_foot;
    feet_jacobians_[RH] = &jacobians_.fr_trunk_J_RH_foot;

    // positive left, negative right
    haa_sign_flip[LF] = 1;
    haa_sign_flip[RF] = -1;
    haa_sign_flip[LH] = 1;
    haa_sign_flip[RH] = -1;

    // setting backward knee for all legs, by default
    kfe_sign_flip[LF] = -1;
    kfe_sign_flip[RF] = -1;
    kfe_sign_flip[LH] = -1;
    kfe_sign_flip[RH] = -1;

    // positive front, negative back
    haaXOffset_sign_flip[LF] = +1;
    haaXOffset_sign_flip[RF] = +1;
    haaXOffset_sign_flip[LH] = -1;
    haaXOffset_sign_flip[RH] = -1;

    // positive left, negative right
    haaYOffset_sign_flip[LF] = +1;
    haaYOffset_sign_flip[RF] = -1;
    haaYOffset_sign_flip[LH] = +1;
    haaYOffset_sign_flip[RH] = -1;
}

InverseKinematics::~InverseKinematics() {
    // should we free the JFootXX members?
}

bool InverseKinematics::getJointPosition(const FootPosition &foot_position,
                                         const LegID& leg_id,
                                         LegJointState& q_leg,
                                         bool clamp,
                                         LegJointBool &q_violation_leg) const {
    double a1 = default_pg_.getValue_haa_hfe();
    double a2 = default_pg_.getValue_upper_leg();
    double a3 = default_pg_.getValue_lower_leg();

    JointIdentifiers jid = toJointID(leg_id, KFE);
    // taking the min and max absolute value of the knee joint angle
    // from the joint limits
    double min_knee_angle = std::min(std::abs(q_min_(jid)), std::abs(q_max_(jid)));
    double max_knee_angle = std::max(std::abs(q_min_(jid)), std::abs(q_max_(jid)));

    double p_x, p_y, p_z;
    //double R;   // TODO: Not used. Commented to remove warning
    double r;
    double haa2foot, hfe2foot, shorten_factor;
    double hfe2foot_min, hfe2foot_max, haa2foot_min, haa2foot_max;
    double sin_alpha, cos_alpha, cos_beta, sin_beta;

    iit::dog::JointState joints_position;
    joints_position.setZero();
    Eigen::Matrix<double, 3, 1> hip_position;
    Eigen::Matrix<double, 3, 1> delta_foot_position;

    hfe2foot_min = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - max_knee_angle));
    hfe2foot_max = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - min_knee_angle));
    haa2foot_min = sqrt(a1 * a1 + hfe2foot_min * hfe2foot_min);
    haa2foot_max = sqrt(a1 * a1 + hfe2foot_max * hfe2foot_max);

    hip_position(0) = haaXOffset_sign_flip[leg_id] * default_pg_.getValue_haa_x();
    hip_position(1) = haaYOffset_sign_flip[leg_id] * default_pg_.getValue_haa_y();
    hip_position(2) = 0.0; // assuming the hip is always at zero level!!

    delta_foot_position = foot_position - hip_position;

    haa2foot = delta_foot_position.norm();

    //Workspace check regarding maximum leg extension
    //It does not consider HAA and HFE joint limits
    if(haa2foot < haa2foot_min) {
        shorten_factor = haa2foot_min / haa2foot;
    } else if(haa2foot > haa2foot_max) {
        shorten_factor = haa2foot_max / haa2foot;
    } else {
        shorten_factor = 1.0;
    }

    p_x = shorten_factor * delta_foot_position(0);
    p_y = shorten_factor * delta_foot_position(1);
    p_z = shorten_factor * delta_foot_position(2);

    //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
    r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

    q_leg(0) = -atan2(haa_sign_flip[leg_id] * p_y * r + p_z * a1, - p_z * r +
                      haa_sign_flip[leg_id] * p_y * a1);

    hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
    sin_alpha = - p_x / hfe2foot;
    cos_alpha = r / hfe2foot;
    cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
    sin_beta = sqrt(1 - cos_beta * cos_beta);

    q_leg(1) = atan2(sin_alpha * cos_beta -kfe_sign_flip[leg_id] * cos_alpha
                     * sin_beta, cos_alpha * cos_beta + kfe_sign_flip[leg_id]
                     * sin_alpha * sin_beta);

    q_leg(2) = -kfe_sign_flip[leg_id]
               * acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3)
                      / (2 * a2 * a3)) + kfe_sign_flip[leg_id] * M_PI;

    //Check if the outputs are inf or nan
    for (int joint = LegJoints::HAA; joint <= LegJoints::KFE; joint++) {
        if (!std::isfinite(q_leg(joint))) {
            //std::cerr << "Position of joint " << joint << " and leg " << leg_id
            //          << " is not finite!" << std::endl;
            return false;
        }
    }

    for (int leg_joint = HAA; leg_joint <= KFE; leg_joint++) {
        q_violation_leg(leg_joint) = false;
        if (q_leg(leg_joint) > getLegJointState(leg_id, q_max_)(leg_joint)) {
            if (clamp) {
                q_leg(leg_joint) = getLegJointState(leg_id, q_max_)(leg_joint);
            }
            q_violation_leg(leg_joint) = true;
        }

        if (q_leg(leg_joint) < getLegJointState(leg_id, q_min_)(leg_joint)) {
            if(clamp) {
                q_leg(leg_joint) = getLegJointState(leg_id, q_min_)(leg_joint);
            }
            q_violation_leg(leg_joint) = true;
        }
    }
    return true;
}

bool InverseKinematics::getJointVelocity(const FootVelocity &foot_velocity,
        const LegID &leg_id,
        const Eigen::Vector3d &q_leg,
        Eigen::Vector3d &qd_leg) {

    JointState q_dummy;
    q_dummy.setZero();
    setLegJointState(leg_id, q_leg, q_dummy);

    jacobians_.updateParameters();//just in case the getter has changed...
    jacobians_.fr_trunk_J_LF_foot(q_dummy);
    jacobians_.fr_trunk_J_RF_foot(q_dummy);
    jacobians_.fr_trunk_J_LH_foot(q_dummy);
    jacobians_.fr_trunk_J_RH_foot(q_dummy);

    LegJointState my_qd_leg = (*feet_jacobians_[leg_id]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

    if(my_qd_leg.allFinite()){
        qd_leg = my_qd_leg;
        return true;
    } else {
        std::cerr << "[InverseKinematics::getJointVelocity] A joint velocity ";
        std::cerr << "for leg "<< leg_id << " is not finite!" << std::endl;
    }

    return true;
}

bool InverseKinematics::getJointState(const FootPosition& foot_position,
                                      const FootVelocity& foot_velocity,
                                      const FootAcceleration &foot_acceleration,
                                      const dog::LegID &leg_id,
                                      Eigen::Vector3d &q_leg,
                                      Eigen::Vector3d &qd_leg,
                                      Eigen::Vector3d &qdd_leg,
                                      bool clamp,
                                      dog::LegJointBool& q_violation_leg) {
    // if we can compute position
    if(getJointPosition(foot_position, leg_id, q_leg, clamp, q_violation_leg)) {
        // and we can compute velocity
        if(getJointVelocity(foot_velocity, leg_id, q_leg, qd_leg)) {
            // compute inverse kinematics for joint accelerations
            qdd_leg = (*feet_jacobians_[leg_id]).block<3, 3>(rbd::LX, 0).inverse() *
                      (foot_acceleration - (1.0 / dt_) * ((*feet_jacobians_[leg_id]).block<3, 3>(rbd::LX, 0)
                              - old_feet_jacobians_[leg_id]) * qd_leg);

            old_feet_jacobians_[leg_id] = (*feet_jacobians_[leg_id]).block<3, 3>(rbd::LX, 0);

            //Check if the outputs are inf or nan
            for (int joint = dog::LegJoints::HAA; joint <= dog::LegJoints::KFE; joint++) {
                if (!std::isfinite(qdd_leg(joint))) {
                    std::cerr << "Acceleration of joint " << joint << " and leg " << leg_id
                              << " is not finite!" << std::endl;
                    return false;
                }
            }
            return true;
        }
        return false;
    }
    return false;
}

bool InverseKinematics::getJointState(const FootPositions &foot_positions,
                                      const FootVelocities &foot_velocities,
                                      const FootAccelerations &foot_accelerations,
                                      JointState &q,
                                      JointState &qd,
                                      JointState &qdd,
                                      bool clamp) {
    bool ret = true;
    for(int leg = dog::LF; leg<= dog::RH; leg++){
        LegJointState q_leg;
        LegJointState qd_leg;
        LegJointState qdd_leg;

        ret = ret && getJointState(foot_positions[leg],
                                   foot_velocities[leg],
                                   foot_accelerations[leg],
                                   LegID(leg),
                                   q_leg, qd_leg, qdd_leg, clamp);
        setLegJointState(LegID(leg),q_leg,q);
        setLegJointState(LegID(leg),qd_leg,qd);
        setLegJointState(LegID(leg),qdd_leg,qdd);
    }
    return ret;
}

bool InverseKinematics::getJointState(const FootPositions &foot_positions,
                                           const FootVelocities &foot_velocities,
                                           const FootAccelerations &foot_accelerations,
                                           JointState &q,
                                           JointState &qd,
                                           JointState &qdd,
                                           bool clamp,
                                           dog::JointBoolMap& q_violation)
{
    bool ret = true;
    for(int leg = dog::LF; leg<=RH; leg++){
        LegJointState q_leg;
        LegJointState qd_leg;
        LegJointState qdd_leg;
        LegJointBool q_violation_leg;
        ret = ret && getJointState(foot_positions[leg],
                                   foot_velocities[leg],
                                   foot_accelerations[leg],
                                   LegID(leg),
                                   q_leg, qd_leg, qdd_leg, clamp, q_violation_leg);

        setLegJointState(LegID(leg), q_leg, q);
        setLegJointState(LegID(leg), qd_leg, qd);
        setLegJointState(LegID(leg), qdd_leg, qdd);

        for(int leg_link = HAA; leg_link <= KFE; leg_link++){
            q_violation[toJointID(LegID(leg),LegJoints(leg_link))] =
                    q_violation_leg(LegJoints(leg_link));
        }

    }
    return ret;
}

void InverseKinematics::setKneeConfiguration(const LegBoolMap &is_knee_backward) {
    for(int leg = LF; leg <= RH; leg++) {
        is_knee_backward[leg] ? kfe_sign_flip[leg] = -1 : kfe_sign_flip[leg] = 1;
    }
}

// -------- DEPRECATED FUNCTIONS BELOW ----------
dog::LegBoolMap InverseKinematics::calculate(const dog::LegDataMap<rbd::Vector3d>& des_foot_pos,
        const dog::LegDataMap<rbd::Vector3d>& des_foot_vel,
        const dog::LegDataMap<rbd::Vector3d>& des_foot_accel,
        dog::LegDataMap<rbd::Vector3d>& des_joint_pos,
        dog::LegDataMap<rbd::Vector3d>& des_joint_vel,
        dog::LegDataMap<rbd::Vector3d>& des_joint_accel,
        const dog::JointState& q_) {
    static dog::LegDataMap<rbd::Vector3d> aux_foot_pos;
    dog::LegBoolMap joint_range_check(true);

    //Compute inverse kinematics for joint positions
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        singleLegIK(des_foot_pos[leg], des_foot_vel[leg], des_foot_accel[leg], dog::LegID(leg),
                    des_joint_pos[leg], des_joint_vel[leg], des_joint_accel[leg], q_);
    }


    //Check if the outputs are inf or nan
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        for (int joint = dog::LegJoints::HAA; joint <= dog::LegJoints::KFE; joint++) {
            if (!std::isfinite(des_joint_pos[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Position of joint " << joint << " of leg " << leg
                          << " is " << des_joint_pos[leg](joint) << " ." << std::endl;
            }

            if (!std::isfinite(des_joint_vel[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Velocity of joint " << joint << " of leg " << leg
                          << " is " << des_joint_vel[leg](joint) << " ." << std::endl;

            }
            if (!std::isfinite(des_joint_accel[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Acceleration of joint " << joint << " of leg " << leg
                        << " is " << des_joint_accel[leg](joint) << " ." << std::endl;
            }
        }

    }


    return joint_range_check;

}


dog::LegBoolMap InverseKinematics::calculate(const dog::LegDataMap<rbd::Vector3d>& des_foot_pos,
        const dog::LegDataMap<rbd::Vector3d>& des_foot_vel,
        const dog::LegDataMap<rbd::Vector3d>& des_foot_accel,
        dog::LegDataMap<rbd::Vector3d>& des_joint_pos,
        dog::LegDataMap<rbd::Vector3d>& des_joint_vel,
        dog::LegDataMap<rbd::Vector3d>& des_joint_accel,
        const dog::JointState& q_,
        const dog::LegDataMap<int>& knee_bent_backward) {

    static dog::LegDataMap<rbd::Vector3d> aux_foot_pos;
    dog::LegBoolMap joint_range_check(true);

    //Compute inverse kinematics for joint positions
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        singleLegIK(des_foot_pos[leg], des_foot_vel[leg], des_foot_accel[leg], dog::LegID(leg),
                    des_joint_pos[leg], des_joint_vel[leg], des_joint_accel[leg], q_, knee_bent_backward[leg]);
    }


    //Check if the outputs are inf or nan
    for (int leg = dog::LF; leg <= dog::RH; leg++) {
        for (int joint = dog::LegJoints::HAA; joint <= dog::LegJoints::KFE; joint++) {
            if (!std::isfinite(des_joint_pos[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Position of joint " << joint << " of leg " << leg
                          << " is " << des_joint_pos[leg](joint) << " ." << std::endl;
            }

            if (!std::isfinite(des_joint_vel[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Velocity of joint " << joint << " of leg " << leg
                          << " is " << des_joint_vel[leg](joint) << " ." << std::endl;

            }
            if (!std::isfinite(des_joint_accel[leg](joint))) {
                joint_range_check[leg] = false;
                std::cout << "Acceleration of joint " << joint << " of leg " << leg
                        << " is " << des_joint_accel[leg](joint) << " ." << std::endl;
            }
        }

    }


    return joint_range_check;
}

bool InverseKinematics::singleLegIK(const rbd::Vector3d& foot_position,
                                    const rbd::Vector3d& foot_velocity,
                                    const rbd::Vector3d& foot_acceleration,
                                    const iit::dog::LegID leg_id,
                                    rbd::Vector3d& joint_position,
                                    rbd::Vector3d& joint_velocity,
                                    rbd::Vector3d& joint_acceleration,
                                    const dog::JointState& q_) {
    
    q_.size();   // TODO: Not used. Created to remove warning
    bool sanityCheckFlag = true;

    joint_velocity.setZero();
    joint_acceleration.setZero();

    double a1 = rl.get_haa_hfe_dist();
    double a2 = rl.get_upleg_length();
    double a3 = rl.get_lowleg_length();
    double min_knee_angle = 32 * M_PI / 180;
    double max_knee_angle = 165 * M_PI / 180;


    double p_x, p_y, p_z;
    //double R;   // TODO: Not used. Commented to remove warning
    double r;
    double haa2foot, hfe2foot, shorten_factor;
    double hfe2foot_min, hfe2foot_max, haa2foot_min, haa2foot_max;
    double sin_alpha, cos_alpha, cos_beta, sin_beta;


    int HIP, KNEE;
    iit::dog::JointState joints_position;
    joints_position.setZero();
    Eigen::Matrix<double, 3, 1> hip_position;
    Eigen::Matrix<double, 3, 1> delta_foot_position;
    static rbd::Matrix33d last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootRF = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootLH = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootRH = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0);

    hfe2foot_min = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - max_knee_angle));
    hfe2foot_max = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - min_knee_angle));
    haa2foot_min = sqrt(a1 * a1 + hfe2foot_min * hfe2foot_min);
    haa2foot_max = sqrt(a1 * a1 + hfe2foot_max * hfe2foot_max);


    switch(leg_id) {
    case dog::LF:

        HIP = 1;
        KNEE = -1;
        hip_position << rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

        delta_foot_position = foot_position - hip_position;

        haa2foot = delta_foot_position.norm();

        //Workspace check regarding maximum leg extension
        //It does not consider HAA and HFE joint limits
        if(haa2foot < haa2foot_min) {
            shorten_factor = haa2foot_min / haa2foot;
        } else if(haa2foot > haa2foot_max) {
            shorten_factor = haa2foot_max / haa2foot;
        } else {
            shorten_factor = 1.0;
        }

        p_x = shorten_factor * delta_foot_position(0);
        p_y = shorten_factor * delta_foot_position(1);
        p_z = shorten_factor * delta_foot_position(2);

        //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
        r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

        joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

        hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
        sin_alpha = - p_x / hfe2foot;
        cos_alpha =  HIP * r / hfe2foot;
        cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
        sin_beta = sqrt(1 - cos_beta * cos_beta);

        joint_position(1) = atan2(sin_alpha * cos_beta - (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta + (HIP * KNEE) * sin_alpha * sin_beta);
        joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;


        //VELOCITY
        //Updating Jacobian according to desired joint position
        joints_position.block<3, 1>(dog::LF_HAA, 0) = joint_position;
        jacobians_.fr_trunk_J_LF_foot(joints_position);

        joint_velocity = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


        //ACCELERATION
        joint_acceleration = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                             - (double)taskServoRate * ((*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0) - last_JFootLF) * joint_velocity);

        last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: LF leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;

    case dog::RF:

        //POSITION
        HIP = -1;
        KNEE = -1;
        hip_position << rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

        delta_foot_position = foot_position - hip_position;

        haa2foot = delta_foot_position.norm();

        //Workspace check regarding maximum leg extension
        //It does not consider HAA and HFE joint limits
        if(haa2foot < haa2foot_min) {
            shorten_factor = haa2foot_min / haa2foot;
        } else if(haa2foot > haa2foot_max) {
            shorten_factor = haa2foot_max / haa2foot;
        } else {
            shorten_factor = 1.0;
        }

        p_x = shorten_factor * delta_foot_position(0);
        p_y = shorten_factor * delta_foot_position(1);
        p_z = shorten_factor * delta_foot_position(2);

        //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Created to remove warning
        r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

        joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


        hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
        sin_alpha = - p_x / hfe2foot;
        cos_alpha = - HIP * r / hfe2foot;
        cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
        sin_beta = sqrt(1 - cos_beta * cos_beta);

        joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
        joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;


        //VELOCITY
        //Updating Jacobian according to desired joint position
        joints_position.block<3, 1>(dog::RF_HAA, 0) = joint_position;
        jacobians_.fr_trunk_J_RF_foot(joints_position);


        joint_velocity = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


        //ACCELERATION
        joint_acceleration = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                             - (double)taskServoRate * ((*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0) - last_JFootRF) * joint_velocity);

        last_JFootRF = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0);


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).

        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: RF leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;

    case dog::LH:

        //POSITION
        HIP = 1;
        KNEE = 1;
        hip_position << - rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

        delta_foot_position = foot_position - hip_position;

        haa2foot = delta_foot_position.norm();

        //Workspace check regarding maximum leg extension
        //It does not consider HAA and HFE joint limits
        if(haa2foot < haa2foot_min) {
            shorten_factor = haa2foot_min / haa2foot;
        } else if(haa2foot > haa2foot_max) {
            shorten_factor = haa2foot_max / haa2foot;
        } else {
            shorten_factor = 1.0;
        }

        p_x = shorten_factor * delta_foot_position(0);
        p_y = shorten_factor * delta_foot_position(1);
        p_z = shorten_factor * delta_foot_position(2);


       // R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
        r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

        joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

        hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
        sin_alpha = - p_x / hfe2foot;
        cos_alpha =  HIP * r / hfe2foot;
        cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
        sin_beta = sqrt(1 - cos_beta * cos_beta);

        joint_position(1) = atan2(sin_alpha * cos_beta + (-HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (-HIP * KNEE) * sin_alpha * sin_beta);
        joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

        //VELOCITY
        //Updating Jacobian according to desired joint position
        joints_position.block<3, 1>(dog::LH_HAA, 0) = joint_position;
        jacobians_.fr_trunk_J_LH_foot(joints_position);


        joint_velocity = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


        //ACCELERATION
        joint_acceleration = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                             - (double)taskServoRate * ((*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0) - last_JFootLH) * joint_velocity);
        last_JFootLH = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0);

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: LH leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;

    case dog::RH:

        //POSITION
        HIP = -1;
        KNEE = +1;
        hip_position << - rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

        delta_foot_position = foot_position - hip_position;

        haa2foot = delta_foot_position.norm();

        //Workspace check regarding maximum leg extension
        //It does not consider HAA and HFE joint limits
        if(haa2foot < haa2foot_min) {
            shorten_factor = haa2foot_min / haa2foot;
        } else if(haa2foot > haa2foot_max) {
            shorten_factor = haa2foot_max / haa2foot;
        } else {
            shorten_factor = 1.0;
        }

        p_x = shorten_factor * delta_foot_position(0);
        p_y = shorten_factor * delta_foot_position(1);
        p_z = shorten_factor * delta_foot_position(2);

        //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
        r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

        joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


        hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
        sin_alpha = - p_x / hfe2foot;
        cos_alpha = - HIP * r / hfe2foot;
        cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
        sin_beta = sqrt(1 - cos_beta * cos_beta);

        joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
        joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

        //VELOCITY
        //Updating Jacobian according to desired joint position
        joints_position.block<3, 1>(dog::RH_HAA, 0) = joint_position;
        jacobians_.fr_trunk_J_RH_foot(joints_position);


        joint_velocity = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


        //ACCELERATION
        joint_acceleration = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                             - (double)taskServoRate * ((*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0) - last_JFootRH) * joint_velocity);
        last_JFootRH = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0);


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: RH leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;
    }

    return sanityCheckFlag;

}



bool InverseKinematics::singleLegIK(const rbd::Vector3d& foot_position,
                                    const rbd::Vector3d& foot_velocity,
                                    const rbd::Vector3d& foot_acceleration,
                                    const iit::dog::LegID leg_id,
                                    rbd::Vector3d& joint_position,
                                    rbd::Vector3d& joint_velocity,
                                    rbd::Vector3d& joint_acceleration,
                                    const dog::JointState& q_,
                                    const int& knee_bent_backward) {
    q_.size(); // TODO: Not used. Created to remove warning
    bool sanityCheckFlag = true;

    joint_velocity.setZero();
    joint_acceleration.setZero();

    double a1 = rl.get_haa_hfe_dist();
    double a2 = rl.get_upleg_length();
    double a3 = rl.get_lowleg_length();
    double min_knee_angle = 32 * M_PI / 180;
    double max_knee_angle = 165 * M_PI / 180;


    double p_x, p_y, p_z;
    //double R;   // TODO: Not used. Commented to remove warning
    double r;
    double haa2foot, hfe2foot, shorten_factor;
    double hfe2foot_min, hfe2foot_max, haa2foot_min, haa2foot_max;
    double sin_alpha, cos_alpha, cos_beta, sin_beta;


    int HIP, KNEE;
    iit::dog::JointState joints_position;
    joints_position.setZero();
    Eigen::Matrix<double, 3, 1> hip_position;
    Eigen::Matrix<double, 3, 1> delta_foot_position;
    static rbd::Matrix33d last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootRF = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootLH = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0);
    static rbd::Matrix33d last_JFootRH = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0);

    hfe2foot_min = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - max_knee_angle));
    hfe2foot_max = sqrt(a2 * a2 + a3 * a3 - 2 * a2 * a3 * cos(M_PI - min_knee_angle));
    haa2foot_min = sqrt(a1 * a1 + hfe2foot_min * hfe2foot_min);
    haa2foot_max = sqrt(a1 * a1 + hfe2foot_max * hfe2foot_max);


    switch(leg_id) {
    case dog::LF:

        if(knee_bent_backward) {

            //POSITION
            HIP = 1;
            KNEE = -1;
            hip_position << rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha =  HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta - (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta + (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;


            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::LF_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_LF_foot(joints_position);

            joint_velocity = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0) - last_JFootLF) * joint_velocity);

            last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);

        } else {

            //POSITION
            HIP = 1;
            KNEE = 1;
            hip_position << rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);


            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha =  HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (-HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (-HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::LF_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_LF_foot(joints_position);

            joint_velocity = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0) - last_JFootLF) * joint_velocity);
            last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);

        }


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: LF leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }

        break;

    case dog::RF:

        if(knee_bent_backward) {

            //POSITION
            HIP = -1;
            KNEE = -1;
            hip_position << rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha = - HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::RF_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_RF_foot(joints_position);

            joint_velocity = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0) - last_JFootRF) * joint_velocity);

            last_JFootRF = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0);

        } else {

            //POSITION
            HIP = -1;
            KNEE = +1;
            hip_position << rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha = - HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::RF_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_RF_foot(joints_position);

            joint_velocity = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0) - last_JFootRF) * joint_velocity);
            last_JFootRF = (*feet_jacobians_[RF]).block<3, 3>(rbd::LX, 0);

        }


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).

        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: RF leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;

    case dog::LH:

        if(knee_bent_backward) {
            HIP = 1;
            KNEE = -1;
            hip_position << - rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha =  HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta - (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta + (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;


            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::LF_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_LF_foot(joints_position);

            joint_velocity = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;


            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0) - last_JFootLF) * joint_velocity);

            last_JFootLF = (*feet_jacobians_[LF]).block<3, 3>(rbd::LX, 0);

        } else {

            //POSITION
            HIP = 1;
            KNEE = 1;
            hip_position << - rl.get_haa_x(), rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);


            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, - HIP * p_z * r + p_y * a1);

            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha =  HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (-HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (-HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::LH_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_LH_foot(joints_position);

            joint_velocity = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0) - last_JFootLH) * joint_velocity);
            last_JFootLH = (*feet_jacobians_[LH]).block<3, 3>(rbd::LX, 0);

        }


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: LH leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;

    case dog::RH:

        if(knee_bent_backward) {

            //POSITION
            HIP = -1;
            KNEE = -1;
            hip_position << - rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha = - HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) - M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::RH_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_RH_foot(joints_position);

            joint_velocity = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0) - last_JFootRH) * joint_velocity);

            last_JFootRH = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0);

        } else {

            //POSITION
            HIP = -1;
            KNEE = +1;
            hip_position << - rl.get_haa_x(), - rl.get_haa_y(), rl.get_haa_z();

            delta_foot_position = foot_position - hip_position;

            haa2foot = delta_foot_position.norm();

            //Workspace check regarding maximum leg extension
            //It does not consider HAA and HFE joint limits
            if(haa2foot < haa2foot_min) {
                shorten_factor = haa2foot_min / haa2foot;
            } else if(haa2foot > haa2foot_max) {
                shorten_factor = haa2foot_max / haa2foot;
            } else {
                shorten_factor = 1.0;
            }

            p_x = shorten_factor * delta_foot_position(0);
            p_y = shorten_factor * delta_foot_position(1);
            p_z = shorten_factor * delta_foot_position(2);

            //R = sqrt( p_y * p_y + p_z * p_z);   // TODO: Not used. Commented to remove warning
            r = sqrt( p_y * p_y + p_z * p_z - a1 * a1);

            joint_position(0) = -atan2( HIP * p_y * r + p_z * a1, HIP * p_z * r - p_y * a1);


            hfe2foot = sqrt( p_x * p_x + p_y * p_y + p_z * p_z  - a1 * a1);
            sin_alpha = - p_x / hfe2foot;
            cos_alpha = - HIP * r / hfe2foot;
            cos_beta = (a2 * a2 + hfe2foot * hfe2foot - a3 * a3) / (2 * a2 * hfe2foot);
            sin_beta = sqrt(1 - cos_beta * cos_beta);

            joint_position(1) = atan2(sin_alpha * cos_beta + (HIP * KNEE) * cos_alpha * sin_beta, cos_alpha * cos_beta - (HIP * KNEE) * sin_alpha * sin_beta);
            joint_position(2) = - acos(-(hfe2foot * hfe2foot - a2 * a2 - a3 * a3) / (2 * a2 * a3)) + M_PI;

            //VELOCITY
            //Updating Jacobian according to desired joint position
            joints_position.block<3, 1>(dog::RH_HAA, 0) = joint_position;
            jacobians_.fr_trunk_J_RH_foot(joints_position);

            joint_velocity = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * foot_velocity;

            //ACCELERATION
            joint_acceleration = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0).inverse() * (foot_acceleration
                                 - (double)taskServoRate * ((*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0) - last_JFootRH) * joint_velocity);
            last_JFootRH = (*feet_jacobians_[RH]).block<3, 3>(rbd::LX, 0);

        }


        //Block foot motion in case the desired position is outside the leg worspace
        if(shorten_factor != 1.0) {
            joint_velocity.setZero();
            joint_acceleration.setZero();
        }

        //Check if all the outputs are OK (infinity or not a number).
        for (int i = 0; i < 3; i++) {
            if(!std::isfinite(joint_position(i)) ||
                    !std::isfinite(joint_velocity(i)) ||
                    !std::isfinite(joint_acceleration(i)) ) {
                sanityCheckFlag = false;
                break;
            }
        }
        if(!sanityCheckFlag && false) {
            std::cout << "WARNING: RH leg sanity check failed!" << std::endl;
            std::cout << "joint_position = "
                      << joint_position.transpose() << std::endl;
            std::cout << "joint_velocity = "
                      << joint_velocity.transpose() << std::endl;
            std::cout << "joint_acceleration = "
                      << joint_acceleration.transpose() << std::endl;
        }
        break;
    }

    return sanityCheckFlag;

}


} // namespace Aliengo
} // namespace iit
