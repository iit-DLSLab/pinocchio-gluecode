#ifndef _ALIENGOLIB_CREX_HPP_
#define _ALIENGOLIB_CREX_HPP_

#include <robotlib/robot.hpp>
#include <robotlib/limb.hpp>
#include "aliengo_leg.hpp"
#include "types.hpp"
#include "utils.hpp"
//TODO: robcogen
// #include "robcogen/jacobians.h"

// ROBCOGEN INCLUDES
#include "robcogen/transforms.h"
#include "robcogen/inverse_kinematics.h"
// #include <robcogen/inertia_properties.h>
// #include <robcogen/inverse_dynamics.h>

#include <urdf/model.h>


namespace aliengolib
{
    const int NJOINTS_TOT = 12;
    const int NLINKS_TOT = 12;
    const int NLEGS = 4;
    const int NARMS = 0;
    class Aliengo : public robotlib::Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Aliengo(const std::shared_ptr<robotlib::Trunk> trunk,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms);
        virtual ~Aliengo();

        Eigen::Vector3d getFramePosition(const robotlib::RobotBase::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
            return Eigen::Vector3d().setZero();
        };

        Eigen::Vector3d getFramePosition(const robotlib::RobotBase::JointDataMap<double> &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
            return Eigen::Vector3d().setZero();
        };

        Eigen::Matrix3d getFrameOrientation(const robotlib::RobotBase::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
            return Eigen::Matrix3d().setZero();
        };

        Eigen::Matrix3d getFrameOrientation(const robotlib::RobotBase::JointDataMap<double> &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
            return Eigen::Matrix3d().setZero();
        };

        Eigen::Matrix4d getFramePose(const robotlib::RobotBase::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> origin,
                                        const std::shared_ptr<robotlib::Frame> destination) override
        {
            Eigen::Matrix4d frame_pose{};
            frame_pose.setZero();

            frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
            frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
            frame_pose.row(3) << 0, 0, 0, 1;

            return frame_pose;
        };

        Eigen::Matrix4d getFramePose(const robotlib::RobotBase::JointDataMap<double> &q,
                                        const std::shared_ptr<robotlib::Frame> origin,
                                        const std::shared_ptr<robotlib::Frame> destination) override
        {
            Eigen::Matrix4d frame_pose{};
            frame_pose.setZero();

            frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
            frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
            frame_pose.row(3) << 0, 0, 0, 1;

            return frame_pose;
        };

        Eigen::Vector3d getFootPosition(const robotlib::RobotBase::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFramePosition(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Vector3d getFootPosition(const robotlib::RobotBase::JointDataMap<double> &q,
                                        const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFramePosition(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::RobotBase::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::RobotBase::JointDataMap<double> &q,
                                            const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix4d getFootPose(const robotlib::RobotBase::JointState &q,
                                    const std::shared_ptr<robotlib::Frame> foot) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

        Eigen::Matrix4d getFootPose(const robotlib::RobotBase::JointDataMap<double> &q,
                                    const std::shared_ptr<robotlib::Frame> foot) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

        void getFootPosition(const robotlib::RobotBase::JointState &q,
                                const std::shared_ptr<robotlib::LimbBase> leg,
                                Eigen::Vector3d &footPos) override
        {
            footPos = this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        void getFootPosition(const robotlib::RobotBase::JointDataMap<double> &q,
                                const std::shared_ptr<robotlib::LimbBase> leg,
                                Eigen::Vector3d &footPos) override
        {
            footPos = this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::RobotBase::JointState &q,
                                            const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::RobotBase::JointDataMap<double> &q,
                                            const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        Eigen::Matrix4d getFootPose(const robotlib::RobotBase::JointState &q,
                                    const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

        Eigen::Matrix4d getFootPose(const robotlib::RobotBase::JointDataMap<double> &q,
                                    const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

        virtual void getFootJacobian(const robotlib::RobotBase::JointState &q,
                                        const std::shared_ptr<robotlib::LimbBase> leg,
                                        Jacobian &footJac) override
        {
            footJac.setOnes();
        };

        virtual void getFootJacobian(const robotlib::RobotBase::JointDataMap<double> &q,
                                        const std::shared_ptr<robotlib::LimbBase> leg,
                                        Jacobian &footJac) override
        {
            footJac.setOnes();
        };

		virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) override;

        LegDataMap<std::shared_ptr<robotlib::Frame>> getFeet() override
        {
            auto feet = this->makeLegDataMap<std::shared_ptr<robotlib::Frame>>();

            for (auto leg : *(this->getLegs()))
            {
                feet[leg] = std::make_shared<robotlib::Link>("link");
            }

            return feet;
        };

        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                const robotlib::RobotBase::JointState &joint_velocity,
                                const robotlib::RobotBase::JointState &joint_acceleration,
                                LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) override;

        virtual void forwardKinematics(const Eigen::Vector3d &joint_position,
                                const Eigen::Vector3d &joint_velocity,
                                const Eigen::Vector3d &joint_acceleration,
                                Eigen::Vector3d &end_effector_position,
                                Eigen::Vector3d &end_effector_velocity,
                                Eigen::Vector3d &end_effector_acceleration,
                                const std::shared_ptr<robotlib::Frame> end_effector) override
        {
            std::cout << "Forward Kinematics 2" << std::endl;
        };

        virtual void inverseKinematics(const Eigen::Vector3d &end_effector_position,
                                const Eigen::Vector3d &end_effector_velocity,
                                const Eigen::Vector3d &end_effector_acceleration,
                                Eigen::Vector3d &joint_position,
                                Eigen::Vector3d &joint_velocity,
                                Eigen::Vector3d &joint_acceleration,
                                const std::shared_ptr<robotlib::Frame> end_effector) override
        {
            std::cout << "Inverse Kinematics 1" << std::endl;
        };

        virtual void inverseKinematics(const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                robotlib::RobotBase::JointState &joint_position,
                                robotlib::RobotBase::JointState &joint_velocity,
                                robotlib::RobotBase::JointState &joint_acceleration) override;

        virtual void inverseKinematics(const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       const robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> &robot_jacobian,
                                       robotlib::RobotBase::JointState &joint_position,
                                       robotlib::RobotBase::JointState &joint_velocity,
                                       robotlib::RobotBase::JointState &joint_acceleration) override;

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                const JointState &joint_position,
                                const JointState &joint_velocity,
                                const JointState &joint_acceleration,
                                Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                JointState &tau_joints);                   ///output
        
        
        double getRobotMass() { return 21.525; } ///TODO: compute total mass from links and trunk masses (this could be done in robotlib)
        Eigen::Vector3d getRobotCoM() { return Eigen::Vector3d().setZero(); }

        
        // void updateLinearJacobian(const robotlib::RobotBase::JointState &joints_positions,
        //                           const rcg::Jacobians &jacobians,
		// 					      robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> &robot_jacobian);

    private:
        // Define kinematic variables
        robotlib::RobotBase::LegDataMap<KinematicsConfig> kinConfig_;	
        robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 3>> b_R_h_;
        robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 3>> h_R_b_;
        robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> hipPos_;
        
        urdf::Model robot_model_;

        //**********  RobCoGen variables  **********
		std::shared_ptr<iit::Aliengo::HomogeneousTransforms> homogeneous_transforms_;
		std::shared_ptr<iit::dog::KinDynParams> param_getter_;
        std::shared_ptr<iit::Aliengo::InverseKinematics> inverse_kinematics_;
        
        
        // void setJointLimitsFromUrdf();
        
        // inv_dyn_.reset(new iit::Aliengo::dyn::InverseDynamics(*aliengo_inertia_props_, *aliengo_motion_transforms_));
		// fwd_kin_.reset(new iit::Aliengo::ForwardKinematics(*robot_params_));
		// feet_jacobians_.reset(new iit::Aliengo::FeetJacobians(*aliengo_jacobians_));
		// shin_jacobians_.reset(new iit::Aliengo::ShinJacobians(*robot_params_));
		// jsim_.reset(new iit::Aliengo::dyn::JSIM(*aliengo_inertia_props_, *aliengo_force_transforms_));
		// inertiaProps_.reset(aliengo_inertia_props_.get());
		// ht_.reset(aliengo_hom_transforms_.get());
		// robot_limits_.reset(new iit::Aliengo::Limits());
		// feet_forces_.reset(new iit::Aliengo::FeetContactForces(*feet_jacobians_, *inv_dyn_, *jsim_));
		// trunk_ctrl_.reset(new iit::dog::TrunkController(*aliengo_hom_transforms_, *aliengo_motion_transforms_, *inv_dyn_, *fwd_kin_, *ik_, *feet_jacobians_, *jsim_, *aliengo_inertia_props_));
        // rcg::MotionTransforms transforms{};
        // rcg::InertiaProperties inertias{};
	    // rcg::InverseDynamics invdyn_;
        // rcg::Jacobians jacobians_;
    };
} //namespace aliengolib

extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t();

#endif // _ALIENGOLIB_CREX_HPP_