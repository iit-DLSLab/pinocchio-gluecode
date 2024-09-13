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

    namespace IK{
      enum TASK{
        POSITION_TASK,
        POSE_TASK
      };
    }

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

        /*!
        * @brief Inverse kinematics. Does not consider the floating base joint. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] frame_name name of the frame.
        * @param[in] q_init_guess initial guess for the joint angles.
        * @param[in] position_des desired position of the frame expressed in base frame.
        * @param[out] q_des desired joint angles.
        */
        virtual void fixedBaseInverseKinematics(const std::string &frame_name,
                                                const robotlib::JointState &q_init_guess,
                                                const Eigen::Vector3d &position_des,
                                                robotlib::JointState &q_des) override;

        /*!
        * @brief Inverse kinematics considering all the legs. Does not consider the floating base joint. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] q_init_guess initial guess for the joint angles.
        * @param[in] positions_des desired positions of all the legs expressed in base frame.
        * @param[out] q_des desired joint angles.
        */
        virtual void fixedBaseInverseKinematics(const robotlib::JointState &q_init_guess,
                                                const robotlib::LegDataMap<Eigen::Vector3d> &positions_des,
                                                robotlib::JointState &q_des) override;
        
        /*!
        * @brief Inverse differential kinematics. Does not consider the floating base joint. It computes the joint velocities from the desired frame linear velocity expressed in base frame. Redundancy is not handled yet.
        * @param[in] frame_name name of the frame
        * @param[in] q joint angles
        * @param[in] velocity_des desired frame linear velocity expressed in base frame
        * @param[out] qd_des desired joint velocities
        */
        virtual void fixedBaseInverseDiffKinematics(const std::string &frame_name,
                                                            const robotlib::JointState &q,
                                                            const Eigen::Vector3d &velocity_des,
                                                            robotlib::JointState &qd_des) override;
        
        /*!
        * @brief Inverse differential kinematics for all the legs. Does not consider the floating base joint. It computes the joint velocities from the desired frame linear velocity expressed in base frame. Redundancy is not handled yet.
        * @param[in] q joint angles
        * @param[in] velocity_des desired linear velocities of all the legs expressed in base frame
        * @param[out] qd_des desired joint velocities
        */                                       
        virtual void fixedBaseInverseDiffKinematics(const robotlib::JointState &q,
                                                    const robotlib::LegDataMap<Eigen::Vector3d> &velocities_des,
                                                    robotlib::JointState &qd_des) override;
        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque at each joint. Before calling this function, you need to call forwardKinematics first
         * Use cases:
         * - robot gravity compensation: robot_velocity = 0, robot_acceleration = 0, joint_velocity = 0, joint_acceleration = 0, f_contact = forces to substain robot weight.
         * - leg gravity compensation: robot_velocity = 0, robot_acceleration = 0, joint_velocity = 0, joint_acceleration = 0, f_contact = 0.
         * - realize desired contact forces and robot accelerations: 
         *    robot_velocity = actual robot velocity
         *    robot_acceleration = desired robot acceleration
         *    joint_position = actual joint position
         *    joint_velocity = actual joint velocity
         *    joint_acceleration = desired joint acceleration
         *    f_contact = desired contact forces. 
         * 
         * @param[in] robot_pose pose of the robot base in base frame.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[in] f_contact map defined as follows: [contact_frame, contact_force], where contact_force is expressed in base_frame.
         * @param[out] tau_joints torque of each joint.
         */
        virtual void inverseDynamics(
                                const Eigen::Matrix<double, 7, 1> &robot_pose,    // robot base
                                const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                const robotlib::JointState &joint_acceleration,
                                const robotlib::eigen::aligned_map<std::string, Eigen::Vector3d>& f_contact,
                                robotlib::JointState &tau_joints) override;
        
        /*!
          * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
          * @details
          * The robot velocity and acceleration are set to zero by default.
          * @param[in] robot_pose pose of the robot base in world frame.
          * @param[in] robot_velocity velocity of the robot base in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[in] joint_velocity velocity of each joint.
          * @param[out] nle non linear effects.
          */
        virtual void computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        Eigen::Matrix<double, 6, 1> &nle_base,
                                        robotlib::JointState &nle_joints) override;

        /*!
        * @brief Inverse dynamics to compute the Centrifugal, Coriolis and Gravity terms.
        * @details
        * The robot velocity is to zero by default.
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] robot_velocity velocity of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[in] joint_velocity velocity of each joint.
        * @param[out] nle_joints non linear effects acting on the joints.
        */
        virtual void computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        robotlib::JointState &nle_joints) override;
        /*!
          * @brief Compute gravity terms.
          * @details
          * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
          * @param[in] robot_pose pose of the robot base in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] g_base gravity term related to the base.
          * @param[out] g_joints gravity term related to the joints.
          */
        virtual void computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          Eigen::Matrix<double, 6, 1> &g_base,
                                          robotlib::JointState &g_joints) override;
        /*!
        * @brief Compute gravity terms, setting only the one related to the joints.
        * @details
        * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
        * @param[in] robot_pose pose of the robot base in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] g_joints gravity term related to the joints.
        */
        virtual void computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          robotlib::JointState &g_joints) override;
        /*!
         * @brief Get position of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame position expressed in origin one.
         */
        virtual Eigen::Vector3d getFramePosition(const robotlib::JointState &q,
                                                 const std::string origin,
                                                 const std::string destination) override;

        /*!
         * @brief Get orientation of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame orientation expressed in origin one.
         */
        virtual Eigen::Matrix3d getFrameOrientation(const robotlib::JointState &q,
                                                    const std::string origin,
                                                    const std::string destination) override;
        /*!
         * @brief Get pose (in homogeneous coordinates) of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame pose expressed in origin one.
         */
        virtual Eigen::Matrix4d getFramePose(const robotlib::JointState &q,
                                             const std::string origin,
                                             const std::string destination) override;
                                     
		// /*!
    //      * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
    //      * @param[in] q angles of the joints.
    //      * @param[in] foot foot frame.
    //      * @return foot position expressed in trunk frame.
    //      */
    //     virtual Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
    //                                             const std::shared_ptr<robotlib::Frame> foot) override;

		// /*!
    //      * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
    //      * @details
    //      * This function gets the foot corresponding to the leg in input and then it computes the foot position.
    //      * @param[in] q angles of the joints.
    //      * @param[in] leg leg corresponding to the foot.
    //      * @return foot position expressed in trunk frame.
    //      */
    //     virtual Eigen::Vector3d getFootPosition(const robotlib::JointState &q,
    //                                  const std::shared_ptr<robotlib::LimbBase> leg) override;

    //     /*!
    //      * @brief Get foot orientation expressed in trunk frame.
    //      * @param[in] q angles of the joints.
    //      * @param[in] foot foot frame.
    //      * @return foot orientation expressed in trunk frame.
    //      */
    //     virtual Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
    //                                                const std::shared_ptr<robotlib::Frame> foot) override;

    //     /*!
    //      * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
    //      * @details
    //      * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
    //      * @param[in] q angles of the joints.
    //      * @param[in] leg leg corresponding to the foot.
    //      * @return foot orientation expressed in trunk frame.
    //      */
    //     virtual Eigen::Matrix3d getFootOrientation(const robotlib::JointState &q,
    //                                                const std::shared_ptr<robotlib::LimbBase> leg) override;

    //     /*!
    //      * @brief Get foot pose expressed in trunk frame.
    //      * @param[in] q angles of the joints.
    //      * @param[in] foot foot frame.
    //      * @return foot pose expressed in trunk frame.
    //      */
    //     virtual Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
    //                                         const std::shared_ptr<robotlib::Frame> foot) override;

    //     /*!
    //      * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
    //      * @details
    //      * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
    //      * @param[in] q angles of the joints.
    //      * @param[in] leg leg corresponding to the foot.
    //      * @return foot pose expressed in trunk frame.
    //      */
    //     virtual Eigen::Matrix4d getFootPose(const robotlib::JointState &q,
    //                                         const std::shared_ptr<robotlib::LimbBase> leg) override;

        
        /*!
          * @brief Get the geometric jacobian of the frame expressed in base frame, related to the limbs only (so considering the actuated joints). The order is linear_jacobian, angular_jacobian. For a complete jacobian, see getFrameWholeBodyJacobian.
          * @param[in] q angles of the joints.
          * @param[in] frame_name name of the frame.
          * @param[out] jacobian jacobian to be filled.
          */
        virtual void computeLimbsJacobian( const robotlib::JointState &q,
                                    const std::string& frame_name,
                                    Eigen::MatrixXd &jacobian) override;

        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame, related to both robot base and joints. The order is linear_jacobian, angular_jacobian. For a jacobian considering only the joints, see getLimbsJacobian.
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] q angles of the joints.
        * @param[in] frame_name name of the frame.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeWholeBodyJacobian(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &q,
                                        const std::string& frame_name,
                                        Eigen::MatrixXd &jacobian) override;

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
         * @brief Get link mass.
         * @param[in] name name of the link.
         * @return link mass.
         */
        virtual double getLinkMass(const std::string name) const override;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @param[in] q angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d getRobotCoM(const robotlib::JointState q) override;

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
        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const robotlib::JointState &joint_position) override;

        /*!
         * @brief Compute CoM legs contribution in base frame.
         * @param[in] q angles of the joints.
         * @return CoM legs contribution in base frame.
         */
        virtual Eigen::Vector3d getLegContribution(const robotlib::JointState &q) override;

        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] base_pos base position in world frame.
         * @return CoM position in world frame.
         */
        virtual Eigen::Vector3d getCoMFromBase(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos)  override;

        /*!
         * @brief Compute robot base position in world frame, from CoM position in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] com robot CoM postion in world frame.
         * @return base position in world frame.
         */
        virtual Eigen::Vector3d getBaseFromCoM(const robotlib::JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &com)  override;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const robotlib::JointState &q)  override;

        /*!
         * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] offset_com CoM offset in base frame.
         * @return CoM velocity in world frame.
         */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &R,
                                                                 const Eigen::Vector3d offset_com)  override;
                                                      
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const robotlib::JointState &q,
                                                               const robotlib::JointState &qd)  override;

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

        /*!
        * @brief Get the base ID. 
        */
        pinocchio::FrameIndex getBaseID() const;
        /*!
        * @brief Closed loop inverse kinematics.
        * @details
        * This function computes the joint angles using the CLIK algorithm as in Handbook of Robotics, eq. 10.29.
        * Does not consider the floating base joint yet. It computes the joint angles from the desired frame position expressed in base frame. Redundancy is not handled yet.
        * @param[in] frame_name name of the frame
        * @param[in] q_init_guess initial guess of the joint angles
        * @param[in] oMdes desired task pose in pinocchio world frame
        * @param[in] task_type type of the task: IK::TASK::POSITION_TASK or IK::TASK::POSE_TASK
        */
        Eigen::VectorXd clik(const std::string &frame_name,
                        const Eigen::VectorXd &q_init_guess,
                        const pinocchio::SE3 &oMdes,
                        const IK::TASK task_type);

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

        Eigen::VectorXd reorderJoints(const Eigen::VectorXd& data) const;
        Eigen::MatrixXd reorderLimbsJacobian(const Eigen::MatrixXd& jacobian) const;
        Eigen::MatrixXd reorderWholeBodyJacobian(const Eigen::MatrixXd& jacobian) const;

        Eigen::VectorXd fromRobotlibToPinocchioJointState(const robotlib::JointState &joint_position);
        Eigen::VectorXd fromRobotlibToPinocchioJointState(const Eigen::Matrix<double, 7, 1> &robot_pose, const robotlib::JointState &joint_position);
        Eigen::VectorXd fromRobotlibToPinocchioJointVelocity(const robotlib::JointState &joint_velocity);
        Eigen::VectorXd fromRobotlibToPinocchioJointVelocity(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_velocity);


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