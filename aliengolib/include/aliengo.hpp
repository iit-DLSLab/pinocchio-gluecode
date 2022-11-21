/*!
 * @file aliengo.hpp
 *
 * @brief Aliengo class definition and functions prototypes.
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ALIENGOLIB_ALIENGO_HPP_
#define _ALIENGOLIB_ALIENGO_HPP_

#include <robotlib/robot.hpp>
#include <robotlib/limb.hpp>
#include "aliengo_leg.hpp"

#include "utils.hpp"

// ROBCOGEN INCLUDES
#include "robcogen/inverse_kinematics.h"
#include "robcogen/transforms.h"
#include "robcogen/inverse_dynamics.h"
#include "robcogen/inertia_properties.h"

#include <urdf/model.h>

namespace aliengolib
{
    //! Number of joints of the robot.
    const int NJOINTS_TOT = 12;
    //! Number of links of the robot.
    const int NLINKS_TOT = 12;
    //! Number of legs of the robot.
    const int NLEGS = 4;
    //! Number of arms of the robot.
    const int NARMS = 0;

    /*!
     * @brief Aliengo class.
     * @details
     * This class represents the Aliengo robot with a specific number of joints, links, legs and arms. It inherits from the Robot class.
     * @tparam NJOINTS_TOT number of joints of the robot.
     * @tparam NLINKS_TOT number of links of the robot.
     * @tparam NLEGS number of legs of the robot.
     * @tparam NARMS number of arms of the robot.
     */
    class Aliengo : public robotlib::Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] trunk shared pointer pointing to the trunk object.
         * @param[in] legs shared pointer pointing to the robot's legs.
         * @param[in] arms shared pointer pointing to the robot's arms.
         * @param[in] robot_urdf urdf of the robot.
         */
        Aliengo(const std::shared_ptr<robotlib::Trunk> trunk,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms, const std::string& robot_urdf);

        /*!
         * @brief Destructor.
         */
        virtual ~Aliengo();

        /*!
          * @brief Forward kinematics.
          * @details
          * It computes the position of each end effector (foot) expressed in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] end_effector_position position of each end effector (foot) in base frame.
          */
        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) override;

        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector (foot) expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] end_effector_velocity velocity of each end effector (foot) in base frame.
         */
        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                       const robotlib::RobotBase::JointState &joint_velocity,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) override;

        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle, velocity and acceleration of each joint from the position, velocity and acceleration of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
         * @param[in] end_effector_acceleration acceleration of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * @param[out] joint_velocity velocity of each joint.
         * @param[out] joint_acceleration acceleration of each joint.
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) override;

        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle of each joint from the position of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       JointState &joint_position) override;

        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque of each joint and the wrench at the base.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base,
                                     JointState &tau_joints) override;

        /*!
         * @brief Compute gravity terms.
         * @details
         * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      Eigen::Matrix<double, 6, 1> &wrench_base,
                                                      JointState &tau_joints) override;

        // ** GET FUNCTIONS **

        /*!
         * @brief Get position of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame position expressed in origin one.
         */
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<robotlib::Frame> origin,
                                                 const std::shared_ptr<robotlib::Frame> destination) override;

        /*!
         * @brief Get orientation of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame orientation expressed in origin one.
         */
        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<robotlib::Frame> origin,
                                                    const std::shared_ptr<robotlib::Frame> destination) override;

        /*!
         * @brief Get pose of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame pose expressed in origin one.
         */
        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<robotlib::Frame> origin,
                                             const std::shared_ptr<robotlib::Frame> destination) override;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                const std::shared_ptr<robotlib::Frame> foot) override;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot position.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                     const std::shared_ptr<robotlib::LimbBase> leg) override;

        /*!
         * @brief Get foot orientation expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<robotlib::Frame> foot) override;

        /*!
         * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<robotlib::LimbBase> leg) override;

        /*!
         * @brief Get foot pose expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<robotlib::Frame> foot) override;

        /*!
         * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<robotlib::LimbBase> leg) override;

        /*!
         * @brief Update the linear part of the jacobians in input.
         * @details
         * Each jacobian is associated to a leg. So it maps all the velocities of the leg's joints to foot velocity.
         * @param[in] joint_position angles of the joints.
         * @param[out] robot_jacobian a LegDataMap object, associating a jacobian to each leg.
         */
        virtual void updateLinearJacobian(const JointState &joint_position,
                                          LegDataMap<Jacobian> &robot_jacobian) override;

        /*!
         * @brief Get total robot mass.
         * @return total robot mass.
         */
        virtual double getRobotMass() const override;

        /*!
         * @brief Get trunk mass.
         * @return trunk mass.
         */
        virtual double getTrunkMass() const override;

        /*!
         * @brief Get total legs' mass.
         * @return total legs' mass.
         */
        virtual double getLegsMass() const override;

        /*!
         * @brief Get the CoM of the trunk.
         * @return trunk's CoM.
         */
        virtual Eigen::Matrix<double, 3, 1> getTrunkCOM() const override;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_position) override;

        /*!
         * @brief Compute CoM legs contribution in base frame.
         * @param[in] q angles of the joints.
         * @return CoM legs contribution in base frame.
         */
        virtual Eigen::Vector3d getLegContribution(const JointState &q) override;

        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] base_pos base position in world frame.
         * @return CoM position in world frame.
         */
        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) override;

        /*!
         * @brief Compute robot base position in world frame, from CoM position in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] com robot CoM postion in world frame.
         * @return base position in world frame.
         */
        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &com) override;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const JointState &q) override;

        /*!
         * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] offset_com CoM offset in base frame.
         * @return CoM velocity in world frame.
         */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &R,
                                                                 const Eigen::Vector3d offset_com) override;

        // ** SET FUNCTIONS **

        /*!
         * @brief Set trunk's CoM.
         * @param[in] trunk_com CoM of trunk to be set.
         */
        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) override;

        /*!
         * @brief Set trunk's mass.
         * @param[in] trunk_mass mass of trunk to be set.
         */
        virtual void setTrunkMass(const double& trunk_mass) override;

		/*!
		 * @brief Set inverse kinematics time period.
         * @details
         * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
         * @param[in] period period of the controller.
		 */
        virtual void setInvKinTimePeriod(const double& period) override;

    private:
		/*!
		 * @brief Set the joint limits from the urdf file.
		 */
        void setJointLimitsFromUrdf();

        //! Robot model from the urdf file.
        urdf::Model robot_model_;

        //! Auxiliar variable storing the joints of the robot. This variable can help to avoid unnecessary loops.
        std::array<std::shared_ptr<robotlib::Joint>,NJOINTS_TOT> auxiliar_joints_variable_;

        //**********  RobCoGen variables  **********
        //! Homogeneous transforms.
		std::shared_ptr<iit::Aliengo::HomogeneousTransforms> homogeneous_transforms_;
		//! Robot parameters.
        std::shared_ptr<iit::dog::KinDynParams> robot_params_;
        //! Inverse kinematics.
        std::shared_ptr<iit::Aliengo::InverseKinematics> inverse_kinematics_;
		//! Robot inertias.
        std::shared_ptr<iit::Aliengo::dyn::InertiaProperties> inertias_;
        //! Inverse dynamics.
        std::shared_ptr<iit::Aliengo::dyn::InverseDynamics> inverse_dynamics_;
        //! Motion transforms.
        std::shared_ptr<iit::Aliengo::MotionTransforms> motion_transforms_;
        //! Jacobians.
        std::shared_ptr<iit::Aliengo::Jacobians> jacobians_;
    };
} //namespace aliengolib

/*!
* @brief Factory function to load at run-time the glue code, creating a robot object.
* @return shared pointer pointing to the RobotBase object.
*/
extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t();

/*!
* @brief Factory function to load at run-time the glue code, with external urdf in input.
* @param[in] robot_urdf the urdf of the robot in string format.
* @return shared pointer pointing to the RobotBase object.
*/
extern "C" std::shared_ptr<robotlib::RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf);

/*!
* @brief Factory function to destroy the robot object.
* @param[in] robot the robot object.
*/
extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot);

#endif // _ALIENGOLIB_ALIENGO_HPP_