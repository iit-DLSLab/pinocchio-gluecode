#ifndef IIT_ALIENGO_HPP_
#define IIT_ALIENGO_HPP_

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
#include "robcogen/feet_jacobians.h"

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

        Eigen::Vector3d getFramePosition(const robotlib::JointState &q,
                                         const std::shared_ptr<robotlib::Frame> origin,
                                         const std::shared_ptr<robotlib::Frame> destination) override;

        Eigen::Matrix3d getFrameOrientation(const robotlib::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> origin,
                                            const std::shared_ptr<robotlib::Frame> destination) override
        {
        	q.size();
			origin->getName();
			destination->getName();

            return Eigen::Matrix3d().setZero();
        };

        Eigen::Matrix4d getFramePose(const robotlib::JointState &q,
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

        Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFramePosition(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
                                           const std::shared_ptr<robotlib::Frame> foot) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
        };

        Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
                                    const std::shared_ptr<robotlib::Frame> foot) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

         Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
                             const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            return this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
                                           const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
        };

        Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
                                    const std::shared_ptr<robotlib::LimbBase> leg) override
        {
            Eigen::Matrix4d foot_pose{};
            foot_pose.setZero();

            foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
            foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
            foot_pose.row(3) << 0, 0, 0, 1;

            return foot_pose;
        };

        /*!
         * @brief Get the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void getFootJacobian(const robotlib::JointState &q,
                                     const std::shared_ptr<robotlib::LimbBase> leg,
                                     robotlib::Jacobian &footJac) const override;

        /*!
         * @brief Update the linear part of the feet jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
		virtual void updateLinearJacobian(const robotlib::JointState &joints_positions,
                                          robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const override;

        /*!
         * @brief Update the angular part of the feet jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateAngularJacobian(const robotlib::JointState &joints_positions,
                                          robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const override;

        /*!
         * @brief Update the linear part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateLinearFootJacobian(const robotlib::JointState &joints_positions,
                                          const std::shared_ptr<robotlib::LimbBase> leg,
                                          robotlib::Jacobian &footJac) const override;

        /*!
         * @brief Update the angular part of the foot jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @param[out] footJac jacobian to be filled.
         */
        virtual void updateAngularFootJacobian(const robotlib::JointState &q,
                                     const std::shared_ptr<robotlib::LimbBase> leg,
                                     robotlib::Jacobian &footJac) const override;

        robotlib::LegDataMap<std::shared_ptr<robotlib::Frame>> getFeet() override
        {
            auto feet = this->makeLegDataMap<std::shared_ptr<robotlib::Frame>>();
            for (auto leg : *(this->getLegs()))
            {
                feet[leg] = std::make_shared<robotlib::Link>("link");
            }
            std::cout << "TODO - getFeet" << std::endl;
            return feet;
        };

        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position) const override;

        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       const robotlib::JointState &joint_velocity,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity) const override;

        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       const robotlib::JointState &joint_velocity,
                                       const robotlib::JointState &joint_acceleration,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       robotlib::LegDataMap<Eigen::Vector3d> &end_effector_acceleration) const override;

        virtual robotlib::LegDataMap<Eigen::Vector3d> forwardKinematics(const robotlib::JointState &) const ;


        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       robotlib::JointState &joint_position) const override;

        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                       robotlib::JointState &joint_position,
                                       robotlib::JointState &joint_velocity,
                                       robotlib::JointState &joint_acceleration) const override;

        
        virtual robotlib::JointState inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d>&) const override;

        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque of each joint and the wrench at the base. By default, the robot velocity and acceleration are set to 0.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         */
        void inverseDynamics(Eigen::Matrix<double, 6, 1> &wrench_base,
                                    robotlib::JointState &tau_joints,
                                    const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                    const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    const robotlib::JointState &joint_acceleration,
                                    const Eigen::Matrix<double, 6, 1> &robot_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
                                    const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero()) const override;
        
        /*!
         * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
         * @details
         * The robot velocity and acceleration are set to zero by default.
         * @param[out] tau_joints torque of each joint.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         */
        virtual void inverseDynamicsHTerm(  robotlib::JointState &tau_joints,
                                            const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                            const robotlib::JointState &joint_position,
                                            const robotlib::JointState &joint_velocity,
                                            const Eigen::Matrix<double, 6, 1> &robot_velocity = Eigen::Matrix<double, 6, 1>::Zero(),
                                            const Eigen::Matrix<double, 6, 1> &robot_acceleration = Eigen::Matrix<double, 6, 1>::Zero())
                                            const override;
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                const robotlib::JointState &joint_position,
                                                Eigen::Matrix<double, 6, 1> &wrench_base,
                                                robotlib::JointState &tau_joints);
        
        // double getRobotMass() const override { return 21.525; }

        double getRobotMass() const
        {
            return inertias_->getTotalMass();
        }

        // TODO: remove this override once the dynamic_parameter of trunk_ is correctly set
        const Eigen::Vector3d& getTrunkCOM() const
        {
            return inertias_->getCOM_trunk();
        }

        Eigen::Vector3d getRobotCoM() { return Eigen::Vector3d().setZero(); }

        virtual Eigen::Vector3d getWholeBodyCOM() override;

        virtual Eigen::Vector3d getWholeBodyCOM(const robotlib::JointState &joint_state) const override;

        virtual Eigen::Vector3d getCoMFromBase(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) override;

        virtual Eigen::Vector3d getBaseFromCoM(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) override;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const robotlib::JointState &q,
                                                               const robotlib::JointState &qd) override;


        //compute spatial velocity of the CoM (base and joint influence)
        //the twist should be expressed in base frame and the velocity is rotated according to matrix R
        //compute spatial velocity of the CoM (base and joint influence) with update
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const robotlib::JointState &q,
                                                                 const robotlib::JointState &qd) override;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const robotlib::JointState &q) override;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const Eigen::Vector3d offset_com) override;

        virtual Eigen::Vector3d getLegContribution(const robotlib::JointState &q) const override;

        virtual double getTrunkMass() const override;

        virtual double getLegsMass() const override;

        // void updateLinearJacobian(const robotlib::JointState &joints_positions,
        //                           const rcg::Jacobians &jacobians,
		// 					      robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian);

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        Eigen::Matrix4d getImuBaseOffset(const std::string imu_link_name="trunk_imu", const std::string base_link_name="base_link")  const override;

        virtual void setInvKinTimePeriod(const double& period) const override;

        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) const override;
        
        virtual void setTrunkMass(const double& trunk_mass) const override;

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
        std::shared_ptr<iit::Aliengo::FeetJacobians> feet_jacobians_;

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
extern "C" void destroyRobotWithUrdf_t(std::shared_ptr<robotlib::RobotBase>);

#endif // _ALIENGOLIB_ALIENGO_HPP_