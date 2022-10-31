#ifndef _ALIENGOLIB_ALIENGO_HPP_
#define _ALIENGOLIB_ALIENGO_HPP_

#include <robotlib/robot.hpp>
#include <robotlib/limb.hpp>
#include "aliengo_leg.hpp"

#include "utils.hpp"
//TODO: robcogen
// #include "robcogen/jacobians.h"

// ROBCOGEN INCLUDES
#include "robcogen/inverse_kinematics.h"
#include "robcogen/transforms.h"
#include "robcogen/inverse_dynamics.h"
#include "robcogen/inertia_properties.h"

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
        Aliengo(const std::shared_ptr<robotlib::Trunk> trunk,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms, const std::string& robot_urdf);
        virtual ~Aliengo();

        Eigen::Vector3d getFramePosition(const robotlib::RobotBase::JointState &q,
                                         const std::shared_ptr<robotlib::Frame> origin,
                                         const std::shared_ptr<robotlib::Frame> destination) override
        {
        	q.size();
			origin->getName();
			destination->getName();

            return Eigen::Vector3d().setZero();
        };

        Eigen::Matrix3d getFrameOrientation(const robotlib::RobotBase::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
        	q.size();
			origin->getName();
			destination->getName();

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

        Eigen::Vector3d getFootPosition(const robotlib::RobotBase::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFramePosition(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::RobotBase::JointState &q,
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

        void getFootPosition(const robotlib::RobotBase::JointState &q,
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


		virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) override;

        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                                robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) override;

        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                                const robotlib::RobotBase::JointState &joint_velocity,
                                                robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                                robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) override;

        virtual void inverseKinematics(const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                robotlib::RobotBase::JointState &joint_position,
                                robotlib::RobotBase::JointState &joint_velocity,
                                robotlib::RobotBase::JointState &joint_acceleration) override;

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       JointState &joint_position) override;

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                const JointState &joint_position,
                                const JointState &joint_velocity,
                                const JointState &joint_acceleration,
                                Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                JointState &tau_joints);                   ///output
        
        // Hypothesis of fully actuated base
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                const JointState &joint_position,
                                                Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                JointState &tau_joints);              ///output
        
        // double getRobotMass() const override { return 21.525; }

        double getRobotMass() const
        {
            return inertias_->getTotalMass();
        }

        // TODO: remove this override once the dynamic_parameter of trunk_ is correctly set
        const Eigen::Matrix<double, 3, 1>& getTrunkCOM() const
        {
            return inertias_->getCOM_trunk();
        }

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM();

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) override;

        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) override;

        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) override;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &rotationMx,
                                                                const JointState &q) override;
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                            const Eigen::Matrix3d &rotationMx,
                                                            const Eigen::Vector3d offset_com) override;

        virtual Eigen::Vector3d getLegContribution(const JointState &q) override;

        virtual double getTrunkMass() const override;

        virtual double getLegsMass() const override;

        virtual void setInvKinTimePeriod(const double& period) override;

        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) override;
        
        virtual void setTrunkMass(const double& trunk_mass) override;

    private: 
        urdf::Model robot_model_;

        std::array<std::shared_ptr<robotlib::Joint>,NJOINTS_TOT> auxiliar_joints_variable_;
        // std::map<std::shared_ptr<robotlib::Joint>, std::shared_ptr<robotlib::LimbBase>> map_joint_to_limb_;

        //**********  RobCoGen variables  **********
		std::shared_ptr<iit::Aliengo::HomogeneousTransforms> homogeneous_transforms_;
		std::shared_ptr<iit::dog::KinDynParams> robot_params_;
        std::shared_ptr<iit::Aliengo::InverseKinematics> inverse_kinematics_;
		std::shared_ptr<iit::Aliengo::dyn::InertiaProperties> inertias_;
        std::shared_ptr<iit::Aliengo::dyn::InverseDynamics> inverse_dynamics_;
        std::shared_ptr<iit::Aliengo::MotionTransforms> motion_transforms_;
        std::shared_ptr<iit::Aliengo::Jacobians> jacobians_;
		
        void setJointLimitsFromUrdf();
        
        // inv_dyn_.reset(new iit::Aliengo::dyn::InverseDynamics(*aliengo_inertias_, *aliengo_motion_transforms_));
		// fwd_kin_.reset(new iit::Aliengo::ForwardKinematics(*robot_params_));
		// feet_jacobians_.reset(new iit::Aliengo::FeetJacobians(*aliengo_jacobians_));
		// shin_jacobians_.reset(new iit::Aliengo::ShinJacobians(*robot_params_));
		// jsim_.reset(new iit::Aliengo::dyn::JSIM(*aliengo_inertias_, *aliengo_force_transforms_));
		// inertiaProps_.reset(aliengo_inertias_.get());
		// ht_.reset(aliengo_hom_transforms_.get());
		// robot_limits_.reset(new iit::Aliengo::Limits());
		// feet_forces_.reset(new iit::Aliengo::FeetContactForces(*feet_jacobians_, *inv_dyn_, *jsim_));
		// trunk_ctrl_.reset(new iit::dog::TrunkController(*aliengo_hom_transforms_, *aliengo_motion_transforms_, *inv_dyn_, *fwd_kin_, *ik_, *feet_jacobians_, *jsim_, *aliengo_inertias_));
        // rcg::MotionTransforms transforms{};
        // rcg::InertiaProperties inertias{};
	    // rcg::InverseDynamics invdyn_;
        // rcg::Jacobians jacobians_;
    };
} //namespace aliengolib

// TODO
extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t();
extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase>);
extern "C" std::shared_ptr<robotlib::RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf);

#endif // _ALIENGOLIB_ALIENGO_HPP_