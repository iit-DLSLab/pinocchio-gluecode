#ifndef _GLUE_CODE_HPP_
#define _GLUE_CODE_HPP_

#include <robotlib/robot.hpp>
#include <robotlib/limb.hpp>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include <yaml-cpp/yaml.h>

using LimbMap  = std::map<std::string,std::vector<std::string>>;
using LimbList = std::vector<LimbMap>;

namespace glue_code
{
    namespace IK{
      enum TASK{
        POSITION_TASK,
        POSE_TASK
      };
    }

    /*!
     * @brief GlueCode class.
     * @details
     * This class represents the GlueCode robot with a specific number of limbs, joints and links.
    */
    class GlueCode : public robotlib::Robot
    {
    public:
       /*!
         * @brief Constructor.
          * @param[in] kinematics_mapping YAML node containing the kinematics mapping information.
       */
        GlueCode(const YAML::Node& kinematics_mapping);
 		/*!
         * @brief Destructor.
         */
        virtual ~GlueCode();
        /*!
          * @brief Forward kinematics.
          * @details
          * It computes the position of each end effector (foot) expressed in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] end_effector_position position of each end effector (foot) in base frame.
          */
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       robotlib::LimbDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) override;

        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector in base frame.
         * @param[out] end_effector_velocity velocity of each end effector in base frame.
         */
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                       const robotlib::JointState &joint_velocity,
                                       robotlib::LimbDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       robotlib::LimbDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) override;
        
        /*!
        * @brief Forward kinematics.
        * @details
        * It computes the position, orientation, velocity and acceleration of each end effector expressed in base frame.
        * @param[in] joint_position angle of each joint.
        * @param[in] joint_velocity velocity of each joint.
        * @param[in] joint_acceleration acceleration of each joint.
        * @param[out] end_effector_position position of each end effector in base frame.
        * @param[out] end_effector_orientation orientation of each end effector in base frame.
        * @param[out] end_effector_velocity velocity of each end effector in base frame.
        * @param[out] end_effector_acceleration acceleration of each end effector in base frame.
        */
        virtual void forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    const robotlib::JointState &joint_acceleration,
                                    robotlib::LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LimbDataMap<Eigen::Matrix3d> &end_effector_orientation,
                                    robotlib::LimbDataMap<robotlib::Vec6d> &end_effector_velocity,
                                    robotlib::LimbDataMap<robotlib::Vec6d> &end_effector_acceleration) override;

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
                                                const robotlib::LimbDataMap<Eigen::Vector3d> &positions_des,
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
                                                    const robotlib::LimbDataMap<Eigen::Vector3d> &velocities_des,
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
        * @brief Compute the joint-space inertia matrix.
        * @details
        * @param[in] robot_pose pose of the robot base in world frame.
        * @param[in] joint_position angle of each joint.
        * @param[out] jsInertia joint-space inertia matrix.
        */
        virtual void computeJSInertiaMatrix(
                                        const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &joint_position,
                                        Eigen::MatrixXd &jsInertia) override;
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
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Vector3d computeFramePosition(const robotlib::JointState& q,
                                                const robotlib::FramePtr origin,
                                                const robotlib::FramePtr destination) override;

        /*!
         * @brief Get orientation of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix3d computeFrameOrientation(const robotlib::JointState& q,
                                                const robotlib::FramePtr origin,
                                                const robotlib::FramePtr destination) override;

        /*!
         * @brief Get pose of the origin frame expressed in the destination one.
         * @param[in] q angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return origin frame orientation expressed in destination one.
         */
        virtual Eigen::Matrix4d computeFramePose(const robotlib::JointState& q,
                                                const robotlib::FramePtr origin,
                                                const robotlib::FramePtr destination) override;

        
        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame, related to the limbs only (so considering the actuated joints). The order is linear_jacobian, angular_jacobian. For a complete jacobian, see computeWholeBodyJacobian.
        * @param[in] q angles of the joints.
        * @param[in] frame frame used to compute the jacobian.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeLimbsJacobian( const robotlib::JointState &q,
                                    const robotlib::FramePtr frame,
                                    Eigen::MatrixXd &jacobian) override;

        void computeLimbsJacobian(const robotlib::JointState &q,
                                        const std::string& frame_name,
                                        Eigen::MatrixXd &jacobian) override;
        /*!
        * @brief Get the geometric jacobian of the frame expressed in base frame. The order is linear_jacobian, angular_jacobian. For a jacobian considering only the joints, see getLimbsJacobian.
        * @param[in] q angles of the joints.
        * @param[in] frame frame used to compute the jacobian.
        * @param[out] jacobian jacobian to be filled.
        */
        virtual void computeWholeBodyJacobian(const robotlib::JointState &q,
                                        const robotlib::FramePtr frame,
                                        Eigen::MatrixXd &jacobian) override;

        /*!
         * @brief Get total robot mass.
         * @return total robot mass.
         */
        virtual double getRobotMass() const override;
        
        /*!
         * @brief Get link mass.
         * @param[in] link the link
         * @return link mass.
         */
        virtual double getLinkMass(const robotlib::LinkPtr link) const override;

        /*!
         * @brief Get link inertia about the CoM.
         * @param[in] link the link
         * @return link inertia.
        */
        virtual Eigen::Matrix3d getLinkInertia(const robotlib::LinkPtr link) const override;

        /*!
         * @brief Get link CoM in the joint frame(see https://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file).
         * @param[in] link the link 
         * @return link CoM.
        */
        virtual Eigen::Vector3d getLinkCoM(const robotlib::LinkPtr link) const override;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_position angles of the joints.
         * @return whole body CoM in base frame.
         */
        virtual Eigen::Vector3d computeWholeBodyCoM(const robotlib::JointState& q) override;

        /*!
        *@brief Get the IMU pose in base frame.
        *@param[in] imu_link_name name of the link to which the IMU sensor is attached.
        *@param[in] base_link_name name of the base link.
        *@return IMU pose in base frame.
        */
        virtual Eigen::Matrix4d getImuBaseOffset(const std::string& imu_link_name="trunk_imu", const std::string& base_link_name="base_link") const override;


        void getJointJacobianTimeVariation(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_position, const robotlib::JointState &joint_velocity, const std::string &frame_name, Eigen::MatrixXd &jdotV);
        
        void getBaseAcceleration(const robotlib::JointState &joint_position, const robotlib::JointState &joint_velocity, const robotlib::JointState &joint_acceleration, Eigen::Matrix<double, 6, 1> &end_effector_acceleration);

        void computeBaseJacobian(const robotlib::JointState &q, Eigen::MatrixXd &base_jacobian);

        // ** SET FUNCTIONS **

    private: 
		/*!
		 * @brief Set the joint limits from the urdf file.
		 */
        void setJointLimitsFromUrdf();

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

        //! Auxiliar variable storing the joints of the robot. This variable can help to avoid unnecessary loops.
        std::vector<robotlib::JointPtr> auxiliar_joints_variable_;

        // ** PINOCCHIO **
        pinocchio::Model robot_model;
        pinocchio::Data robot_data;

        /*!
         * @brief Get the joint id to access to any variable of dimension nq, when the root_joint is JointModelFreeFlyer.
          * @param[in] joint_name name of the joint.
          * @return joint id.
         */    
        int getJointIdForNq(const std::string &joint_name) const;

        /*!
         * @brief Get the joint id to access to any variable of dimension nv, when the root_joint is JointModelFreeFlyer.
          * @param[in] joint_name name of the joint.
          * @return joint id.
         */    
        int getJointIdForNv(const std::string &joint_name) const;

        /*!
        * @brief Get the base ID. 
        */
        pinocchio::FrameIndex getBaseID() const;
                
        int getJointIdWithoutRoot(const std::string &joint_name) const;

        LimbList loadLimbsDefinition(const YAML::Node& root);

        // dls to urdf name
        std::map<std::string, std::string> dls_to_urdf_joints_name;
        std::map<std::string, std::string> dls_to_urdf_links_name;

        Eigen::MatrixXd fromPinocchioToRobotlibLimbsJacobian(const Eigen::MatrixXd& jacobian) const;
        Eigen::MatrixXd fromPinocchioToRobotlibWholeBodyJacobian(const Eigen::MatrixXd& jacobian) const;
        Eigen::VectorXd fromRobotlibToPinocchioNqData(const robotlib::JointState &joint_position_type);
        Eigen::VectorXd fromRobotlibToPinocchioNqData(const Eigen::Matrix<double, 7, 1> &robot_pose, const robotlib::JointState &joint_position_type);
        Eigen::VectorXd fromRobotlibToPinocchioNvData(const robotlib::JointState &joint_velocity_type);
        Eigen::VectorXd fromRobotlibToPinocchioNvData(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_velocity_type);
        
        void fromPinocchioNqDataToRobotlib(const Eigen::VectorXd &q_pin,
                                                robotlib::JointState &joint_position_type);
        void fromPinocchioNvDataToRobotlib(const Eigen::VectorXd &qd,
                                                robotlib::JointState &joint_velocity_type);
        void checkJointNames() const;
        void checkLinkNames() const;        
        // mapping from 
        std::map<int,std::vector<int>> robotlib_to_pin_joint_position_ids;
        // variable used to populate a pinocchio variable of size nv. In case of continuous joints, pinocchio does not add extra elements to the velocity space
        std::map <int,int> robotlib_to_pin_joint_velocity_ids;

        };
} //namespace glue_code

/*!
* @brief Factory function to load at run-time the glue code, creating a robot object.
* @return shared pointer pointing to the RobotBase object.
*/
extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t(const std::string& robot_type);

/*!
* @brief Factory function to destroy the robot object.
* @param[in] robot the robot object.
*/
extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot);

#endif // _GLUE_CODE_HPP_