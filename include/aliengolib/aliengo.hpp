/*!
 * @file aliengo.hpp
 *
 * @brief Aliengo class definition and functions prototypes.
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

#ifndef _ALIENGOLIB_ALIENGO_HPP_
#define _ALIENGOLIB_ALIENGO_HPP_

#include <robotlib/robot.hpp>
#include <robotlib/limb.hpp>
#include "aliengo_leg.hpp"

#include "utils.hpp"
// #include "robcogen/jacobians.h"

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/fwd.hpp"

// ROBCOGEN INCLUDES
#include "robcogen/inverse_kinematics.h"
#include "robcogen/transforms.h"
#include "robcogen/inverse_dynamics.h"
#include "robcogen/inertia_properties.h"

#include "robcogen/feet_jacobians.h"

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
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) override;

        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector (foot) expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] end_effector_velocity velocity of each end effector (foot) in base frame.
         */
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       const robotlib::JointState &joint_velocity,
                                       robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) override;
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                    const int type) override;
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
        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       robotlib::JointState &joint_position,
                                       robotlib::JointState &joint_velocity,
                                       robotlib::JointState &joint_acceleration) override;

        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle of each joint from the position of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         */
        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       robotlib::JointState &joint_position) override;

        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle and velocity of each joint from the position and velocity of each end effector expressed in base frame.
        *
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * @param[out] joint_velocity velocity of each joint.
         */
        virtual void inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       robotlib::JointState &joint_position,
                                       robotlib::JointState &joint_velocity) override;

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
                                     const robotlib::JointState &joint_position,
                                     const robotlib::JointState &joint_velocity,
                                     const robotlib::JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base,
                                     robotlib::JointState &tau_joints) override;

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
                                           override;

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
                                                      const robotlib::JointState &joint_position,
                                                      Eigen::Matrix<double, 6, 1> &wrench_base,
                                                      robotlib::JointState &tau_joints) const override;


        /*!
         * @brief Compute gravity terms and return only the wrench applied to the base to compensate for gravity.
         * @details
         * Instead of using the full version of the computeGravityCompensation function, you can use this function to only get the desired wrench, without taking care of the gravity compensation torques.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @return wrench applied to the base.
         */
        virtual Eigen::Matrix<double, 6, 1> computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const robotlib::JointState &joint_position) const override;
        
        /*!
         * @brief Compute gravity terms and return only the joint torques compensating for gravity.
         * @details
         * Instead of using the full version of the computeGravityCompensation function, you can use this function to only get the desired joint torques that compensate for gravity, without taking care of the wrench applied to the base. Notice that this function has the joint state as output parameter, because returning a JointState object leads to dynamic memory allocation.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const robotlib::JointState &joint_position,
                                                      robotlib::JointState &tau_joints) const override;

        /*!
         * @brief Get position of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame position expressed in origin one.
         */
        virtual Eigen::Vector3d getFramePosition(const robotlib::JointState &q,
                                                 const std::shared_ptr<robotlib::Frame> origin,
                                                 const std::shared_ptr<robotlib::Frame> destination) const override;

        /*!
         * @brief Get orientation of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame orientation expressed in origin one.
         */
        virtual Eigen::Matrix3d getFrameOrientation(const robotlib::JointState &q,
                                                    const std::shared_ptr<robotlib::Frame> origin,
                                                    const std::shared_ptr<robotlib::Frame> destination) const override;
        /*!
         * @brief Get pose of the destination frame expressed in the origin one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame pose expressed in origin one.
         */
        virtual Eigen::Matrix4d getFramePose(const robotlib::JointState &q,
                                             const std::shared_ptr<robotlib::Frame> origin,
                                             const std::shared_ptr<robotlib::Frame> destination) const override;
                                     
		/*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
                                                const std::shared_ptr<robotlib::Frame> foot) const override;

		/*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot position.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot position expressed in trunk frame.
         */
        virtual Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
                                     const std::shared_ptr<robotlib::LimbBase> leg) const override;

        /*!
         * @brief Get foot orientation expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
                                                   const std::shared_ptr<robotlib::Frame> foot) const override;

        /*!
         * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot orientation expressed in trunk frame.
         */
        virtual Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
                                                   const std::shared_ptr<robotlib::LimbBase> leg) const override;

        /*!
         * @brief Get foot pose expressed in trunk frame.
         * @param[in] q angles of the joints.
         * @param[in] foot foot frame.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
                                            const std::shared_ptr<robotlib::Frame> foot) const override;

        /*!
         * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
         * @param[in] q angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot pose expressed in trunk frame.
         */
        virtual Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
                                            const std::shared_ptr<robotlib::LimbBase> leg) const override;
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
         * @brief Update the angular part of the feet jacobian.
         * @details
         * A reference to a Jacobian instance is passed as parameter and it is set to the foot jacobian values. This avoids returning a new Jacobian object that leads to dynamic memory allocation.
         * @param[in] q angles of the joints.
         * @param[out] robot_jacobian jacobians associated to each foot.
         */
        virtual void updateAngularJacobian(const robotlib::JointState &joints_positions,
                                          robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const override;

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
         * @brief Get robot CoM.
         * @return robot CoM.
         */
        virtual Eigen::Vector3d getRobotCoM() const override;

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
        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const robotlib::JointState &joint_position) const override;

        /*!
         * @brief Compute CoM legs contribution in base frame.
         * @param[in] q angles of the joints.
         * @return CoM legs contribution in base frame.
         */
        virtual Eigen::Vector3d getLegContribution(const robotlib::JointState &q) const override;

        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] base_pos base position in world frame.
         * @return CoM position in world frame.
         */
        virtual Eigen::Vector3d getCoMFromBase(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) const override;

        /*!
         * @brief Compute robot base position in world frame, from CoM position in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] com robot CoM postion in world frame.
         * @return base position in world frame.
         */
        virtual Eigen::Vector3d getBaseFromCoM(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &com) const override;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const robotlib::JointState &q) const override;

        /*!
         * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] offset_com CoM offset in base frame.
         * @return CoM velocity in world frame.
         */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &R,
                                                                 const Eigen::Vector3d offset_com) const override;
                                                      
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const robotlib::JointState &q,
                                                               const robotlib::JointState &qd) const override;

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu", const std::string& base_link_name="base_link") const override;

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
        virtual void setTrunkMass(const double trunk_mass) override;

		/*!
		 * @brief Set inverse kinematics time period.
         * @details
         * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
         * @param[in] period period of the controller.
		 */
        virtual void setInvKinTimePeriod(const double period) override;
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
        //! Feet Jacobians.
        std::shared_ptr<iit::Aliengo::FeetJacobians> feet_jacobians_;

        pinocchio::Model robot_model_pin;
        pinocchio::Data robot_data_pin;
        // Pinocchio order the joints following an alphanumeric order, so we need to map our order to the pinocchio one and viceversa
        // our convention: lf rf lh rh
        // pinocchio one: lf lh rf rh
        std::map<int,int> idx_map = {{3,6},{4,7},{5,8}};

        void reoderJoints(Eigen::VectorXd& data) const;
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