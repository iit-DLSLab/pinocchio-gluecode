#ifndef IIT_ALIENGO_INVERSE_KINEMATICS_H_
#define IIT_ALIENGO_INVERSE_KINEMATICS_H_

#include "aliengolib/dog/inverse_kinematics.h"

#include "aliengolib/robcogen/jacobians.h"
#include "RobotLengths.h"

#include <memory>

namespace iit {
namespace Aliengo {

/**
 * @brief Specialized inverse kinematics for Aliengo.
 * Relies on RobCoGen Jacobian computation.
 */
class InverseKinematics : public dog::InverseKinematics {
public:
    typedef typename dog::LegDataMap<dog::Vector3d> FootPositions;
    typedef typename dog::LegDataMap<dog::Vector3d> FootVelocities;
    typedef typename dog::LegDataMap<dog::Vector3d> FootAccelerations;

    typedef typename dog::Vector3d FootPosition;
    typedef typename dog::Vector3d FootVelocity;
    typedef typename dog::Vector3d FootAcceleration;

    typedef typename dog::JointState JointState;
    typedef typename dog::LegJointState LegJointState;

    typedef typename dog::LegBoolMap LegBoolMap;
    typedef typename dog::JointBoolMap JointBoolMap;
    typedef typename dog::LegJointBool LegJointBool;

    using LegID = dog::LegID;
public:
    InverseKinematics(const dog::KinDynParams& default_pg);
    ~InverseKinematics();
    bool getJointPosition(const FootPosition &foot_position,
                          const dog::LegID &leg_id,
                          LegJointState &q_leg,
                          bool clamp,
                          dog::LegJointBool& q_violation_leg) const;

    inline bool getJointPosition(const FootPosition &foot_position,
                                 const dog::LegID &leg_id,
                                 LegJointState &q_leg,
                                 bool clamp = false) const {
        dog::LegJointBool q_violation_leg = dog::LegJointBool::Constant(false);

        return getJointPosition(foot_position,
                                leg_id,
                                q_leg,
                                clamp,
                                q_violation_leg);
    }



    inline bool getJointPosition(const FootPositions &foot_positions,
                                 JointState &q,
                                 bool clamp,
                                 dog::JointBoolMap& q_violation) const {


        // Iterate over the four legs
        for (int leg = dog::LF; leg <= dog::RH; leg++){
            LegJointState q_leg;
            dog::LegJointBool q_violation_leg = dog::LegJointBool::Constant(false);
            // Get the position for each leg
            if(!getJointPosition(foot_positions[leg], dog::LegID(leg), q_leg, clamp, q_violation_leg)){
                return false;
            }
            // then assign the value for one leg to the full joint state and joint violation
            dog::setLegJointState(dog::LegID(leg), q_leg, q);
            // Iterate over each joint of a leg
            for(int leg_joint = dog::HAA; leg_joint <= dog::KFE; leg_joint++){
                // assuming 3 joints per one leg
                q_violation[dog::JointIdentifiers(leg*3 + leg_joint)] = q_violation_leg(leg_joint);
            }
        }
        return true;
    }

    inline bool getJointPosition(const FootPositions &foot_positions,
                                 JointState &q,
                                 bool clamp = false) const {
        dog::JointBoolMap q_violation;

        return getJointPosition(foot_positions, q, clamp, q_violation);
    }

    bool getJointVelocity(const FootVelocity &foot_velocity,
                          const dog::LegID &leg_id,
                          const LegJointState &q_leg,
                          LegJointState &qd_leg);

    inline bool getJointVelocity(const FootVelocities &foot_velocities,
                                 const JointState &q,
                                 JointState &qd) {

        for (int leg = dog::LF; leg <= dog::RH; leg++){
            LegJointState qd_leg;
            if(!getJointVelocity(foot_velocities[leg], dog::LegID(leg), dog::getLegJointState(dog::LegID(leg), q), qd_leg)){
                return false;
            }
            dog::setLegJointState(dog::LegID(leg), qd_leg, qd);
        }
        return true;

    }


    inline bool getJointState(const FootPosition& foot_position,
                              const FootVelocity& foot_velocity,
                              const dog::LegID& leg_id,
                              LegJointState & q_leg,
                              LegJointState& qd_leg,
                              bool clamp,
                              dog::LegJointBool& q_violation_leg)
    {
        bool pos_result = getJointPosition(foot_position,
                                           leg_id,
                                           q_leg,
                                           clamp,
                                           q_violation_leg);

        return pos_result && getJointVelocity(foot_velocity,
                                              leg_id,
                                              q_leg,
                                              qd_leg);
    }


    inline bool getJointState(const FootPosition& foot_position,
                              const FootVelocity& foot_velocity,
                              const dog::LegID& leg_id,
                              LegJointState & q_leg,
                              LegJointState& qd_leg,
                              bool clamp = false)
    {
        dog::LegJointBool q_violation_leg;

        return getJointState(foot_position,
                             foot_velocity,
                             leg_id,
                             q_leg,
                             qd_leg,
                             clamp,
                             q_violation_leg);
    }

    bool getJointState(const FootPosition& foot_position,
                       const FootVelocity& foot_velocity,
                       const FootAcceleration& foot_acceleration,
                       const dog::LegID& leg,
                       LegJointState& q,
                       LegJointState& qd,
                       LegJointState& qdd,
                       bool clamp,
                       dog::LegJointBool& q_violation_leg);

    inline bool getJointState(const FootPosition& foot_position,
                       const FootVelocity& foot_velocity,
                       const FootAcceleration& foot_acceleration,
                       const dog::LegID& leg,
                       LegJointState& q,
                       LegJointState& qd,
                       LegJointState& qdd,
                       bool clamp = false){
        dog::LegJointBool q_violation_leg;

        return getJointState(foot_position,
                             foot_velocity,
                             foot_acceleration,
                             leg,
                             q,qd,qdd,
                             clamp,
                             q_violation_leg);
    }

    inline bool getJointState(const FootPositions &foot_positions,
                              const FootVelocities &foot_velocities,
                              JointState &q,
                              JointState &qd,
                              bool clamp,
                              dog::JointBoolMap& q_violation)
    {

        bool pos_result = getJointPosition(foot_positions,
                                           q,
                                           clamp,
                                           q_violation);
        const JointState q_const = q;

        return pos_result
                && getJointVelocity(foot_velocities, q_const, qd);
    }


    inline bool getJointState(const FootPositions &foot_positions,
                              const FootVelocities &foot_velocities,
                              JointState &q,
                              JointState &qd,
                              bool clamp = false) {
        bool pos_result = getJointPosition(foot_positions,q, clamp);
        const JointState q_const = q;
        return pos_result
                && getJointVelocity(foot_velocities, q_const, qd);
    }

    bool getJointState(const FootPositions &foot_positions,
                       const FootVelocities &foot_velocities,
                       const FootAccelerations &foot_accelerations,
                       JointState &q,
                       JointState &qd,
                       JointState &qdd,
                       bool clamp,
                       dog::JointBoolMap& q_violation);

    bool getJointState(const FootPositions &foot_positions,
                       const FootVelocities &foot_velocities,
                       const FootAccelerations &foot_accelerations,
                       JointState &q,
                       JointState &qd,
                       JointState &qdd,
                       bool clamp = false);

    inline void setTimePeriod(const double& dt) {
        dt_ = dt;
        taskServoRate = (1.0 / dt);
    }

    inline void setKinematicLimits(const JointState& q_min,
                                   const JointState& q_max)
    {
        q_min_ = q_min;
        q_max_ = q_max;
    }

    void setKneeConfiguration(const dog::LegBoolMap& is_knee_backward);

    /**
     * @brief computes the full inverse kinematics (position, velocity,
     * acceleration for all legs)
     * \deprecated This function is deprecated, use getJointState() instead.
     * @param des_foot_pos
     * @param des_foot_vel
     * @param des_foot_accel
     * @param des_joint_pos
     * @param des_joint_vel
     * @param des_joint_accel
     * @param q_
     * @return
     * @date May 10th, 2017
     * @author Victor Barasuol (victor.barasuol@iit.it)
     */
    dog::LegBoolMap calculate(const dog::LegDataMap<rbd::Vector3d>& des_foot_pos,
                              const dog::LegDataMap<rbd::Vector3d>& des_foot_vel,
                              const dog::LegDataMap<rbd::Vector3d>& des_foot_accel,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_pos,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_vel,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_accel,
                              const dog::JointState& q_);
    /**
     * @brief computes the full inverse kinematics (position, velocity,
     * acceleration for all legs)
     * \deprecated This function is deprecated, use getJointState() instead.
     * @param des_foot_pos
     * @param des_foot_vel
     * @param des_foot_accel
     * @param des_joint_pos
     * @param des_joint_vel
     * @param des_joint_accel
     * @param q_
     * @return
     * @date May 10th, 2017
     * @author Victor Barasuol (victor.barasuol@iit.it)
     */
    dog::LegBoolMap calculate(const dog::LegDataMap<rbd::Vector3d>& des_foot_pos,
                              const dog::LegDataMap<rbd::Vector3d>& des_foot_vel,
                              const dog::LegDataMap<rbd::Vector3d>& des_foot_accel,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_pos,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_vel,
                              dog::LegDataMap<rbd::Vector3d>& des_joint_accel,
                              const dog::JointState& q_,
                              const dog::LegDataMap<int>& knee_bent_backward);

private:
    bool singleLegIK(const rbd::Vector3d& foot_position,
                     const rbd::Vector3d& foot_velocity,
                     const rbd::Vector3d& foot_acceleration,
                     const iit::dog::LegID leg_id,
                     rbd::Vector3d& joint_position,
                     rbd::Vector3d& joint_velocity,
                     rbd::Vector3d& joint_acceleration,
                     const dog::JointState& q_);

    bool singleLegIK(const rbd::Vector3d& foot_position,
                     const rbd::Vector3d& foot_velocity,
                     const rbd::Vector3d& foot_acceleration,
                     const iit::dog::LegID leg_id,
                     rbd::Vector3d& joint_position,
                     rbd::Vector3d& joint_velocity,
                     rbd::Vector3d& joint_acceleration,
                     const dog::JointState &q_,
                     const int &knee_bent_backward);

    double taskServoRate;

    const dog::KinDynParams& default_pg_;

    iit::Aliengo::RobotLengths rl;

    LegBoolMap is_knee_backward_ = LegBoolMap(false);

    dog::LegDataMap<int> haa_sign_flip;
    dog::LegDataMap<int> kfe_sign_flip;
    dog::LegDataMap<int> haaXOffset_sign_flip;
    dog::LegDataMap<int> haaYOffset_sign_flip;

    mutable Aliengo::RobotLengths bodyLengths;
    Aliengo::Jacobians jacobians_;
    double dt_ = 0.004; // default task rate is 4 ms
    // double dt_ = 0.01; // default task rate is 10 ms
    // double dt_ = 0.008; // default task rate is 8 ms
    dog::LegDataMap<rbd::Matrix33d> old_feet_jacobians_;
    dog::LegDataMap<Eigen::Matrix<double, 6,3 >*> feet_jacobians_; // useful alias

    JointState q_min_;
    JointState q_max_;


};

} // namespace Aliengo
} // namespace iit

#endif // _ALIENGO_COMMONS_INVERSE_KINEMATICS_H_
