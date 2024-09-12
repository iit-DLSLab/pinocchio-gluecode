/*!
 * @file aliengo.cpp
 *
 * @brief Aliengo class and functions implementation
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

#include "aliengolib/aliengo.hpp"
#include "aliengolib/aliengo_leg.hpp"
#include "aliengolib/urdf_params_getter.h"

#include "aliengolib/robcogen/utils.h"
#include "robotlib/utils/utils.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/rnea.hpp>

#include "robotlib/utils/eigen_utils.hpp"

#include <filesystem>
#include <fstream>

namespace aliengolib
{

    Aliengo::Aliengo(const std::shared_ptr<robotlib::Trunk> trunk,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms, const std::string& robot_urdf)
        :  Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>(
                "aliengo",
                trunk,
                std::make_shared<const robotlib::Container<std::shared_ptr<robotlib::LimbBase>, NLEGS>>(legs),
                std::make_shared<const robotlib::Container<std::shared_ptr<robotlib::LimbBase>, NARMS>>(arms))
                
    {
        // load pinocchio model from urdf
        const std::string urdf_name = "/usr/include/aliengo_description/urdfs/aliengo.urdf";
        pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), robot_model_pin);
        robot_data_pin = pinocchio::Data(robot_model_pin);

        std::array<std::shared_ptr<robotlib::Joint>, NLEGS> children;

        children[0] = getJoint("LF_HAA");
        children[1] = getJoint("RF_HAA");
        children[2] = getJoint("LH_HAA");
        children[3] = getJoint("RH_HAA");

        setChildrenOfTrunk(std::make_shared<robotlib::Container<std::shared_ptr<robotlib::Joint>, NLEGS>>(children));

        setParentOfLink(trunk_, nullptr);

        for (auto leg : *legs_)
        {
            for (auto joint : *(leg->getJoints()))
            {
                const std::string child_name = leg->jointToChildName(joint);
                setChildOfJoint(joint, getLink(child_name));
                const std::string parent_name = leg->jointToParentName(joint);
                setParentOfJoint(joint, getLink(parent_name));
            }
        }

        for (auto leg : *legs_)
        {
            for (auto link : *(leg->getLinks()))
            {
                const std::string child_name = leg->linkToChildName(link);
                setChildOfLink(link, getJoint(child_name));

                const std::string parent_name = leg->linkToParentName(link);
                setParentOfLink(link, getJoint(parent_name));
            }
        }

        for (auto leg : *legs_)
        {
            for (auto joint : *(leg->getJoints()))
            {
                const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
                auxiliar_joints_variable_[joint_id] = joint;
            }
        }
        if (!robot_model_.initString(robot_urdf))
            std::cout << "Failed to parse urdf file" << std::endl;

        // Getting joint limits from urdf

        robot_params_.reset(new iit::dog::UrdfParamsGetter(robot_model_));
        homogeneous_transforms_.reset(new iit::Aliengo::HomogeneousTransforms(*robot_params_));
        inverse_kinematics_.reset(new iit::Aliengo::InverseKinematics(*robot_params_));

        setJointLimitsFromUrdf();
        // Coverting joint kinematic limits to robcogen joint state
        iit::dog::JointState robcogen_q_min{};
        iit::dog::JointState robcogen_q_max{};

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            robcogen_q_min[joint_id] = RobotBase::getMinJointAngle(joint);
            robcogen_q_max[joint_id] = RobotBase::getMaxJointAngle(joint);
        }

        inverse_kinematics_->setKinematicLimits(robcogen_q_min, robcogen_q_max);
        inertias_.reset(new iit::Aliengo::dyn::InertiaProperties(*robot_params_));

        motion_transforms_.reset(new iit::Aliengo::MotionTransforms(*robot_params_));
        
        jacobians_.reset(new iit::Aliengo::Jacobians(*robot_params_));

        feet_jacobians_.reset(new iit::Aliengo::FeetJacobians(*jacobians_));

        inverse_dynamics_.reset(new iit::Aliengo::dyn::InverseDynamics(*inertias_, *motion_transforms_));
    }

    Aliengo::~Aliengo(){}

    void Aliengo::setJointLimitsFromUrdf()
    {
        
        //Get limits from URDF for position, velocity and effort:
        for(std::pair<std::string, std::shared_ptr<urdf::Joint> > jointPair : robot_model_.joints_)
        {
            if (jointPair.second->type == urdf::Joint::REVOLUTE)
            {
                // for (auto leg : *legs_)
                // {
                //     for (auto joint : *leg->getJoints())
                //     {
                //         // **Transform robotlib joint name to urdf one**
                //         std::string urdf_joint_name{joint->getName()};
                //         std::transform(urdf_joint_name.begin(), urdf_joint_name.end(), urdf_joint_name.begin(), ::tolower);
                //         urdf_joint_name.append("_joint");

                //         if(urdf_joint_name == std::get<0>(jointPair))
                //         {
                //             const double q_min = jointPair.second->limits->lower;
                //             const double q_max = jointPair.second->limits->upper;
                //             const double qd_max = jointPair.second->limits->velocity;
                //             const double tau_max = jointPair.second->limits->effort;

                //             setJointLimits(joint, q_min, q_max, qd_max, tau_max);
                            
                //             std::cout << "Set limits for joint " << urdf_joint_name << ":" << std::endl;
                //             std::cout << "q_min = " << q_min << std::endl;
                //             std::cout << "q_max = " << q_max << std::endl;
                //             std::cout << "qd_max = " << qd_max << std::endl; 
                //             std::cout << "tau_max = " << tau_max << std::endl;
                //             std::cout << "\n";
                //         }
                //     }
                    
                // }
                for(auto joint : auxiliar_joints_variable_)
                {
                    // **Transform robotlib joint name to urdf one**
                    std::string urdf_joint_name{joint->getName()};
                    std::transform(urdf_joint_name.begin(), urdf_joint_name.end(), urdf_joint_name.begin(), ::tolower);
                    urdf_joint_name.append("_joint");

                    if(urdf_joint_name == std::get<0>(jointPair))
                    {
                        const double q_min = jointPair.second->limits->lower;
                        const double q_max = jointPair.second->limits->upper;
                        const double qd_max = jointPair.second->limits->velocity;
                        const double tau_max = jointPair.second->limits->effort;

                        setJointLimits(joint, q_min, q_max, qd_max, tau_max);
                        
                        // std::cout << "Set limits for joint " << urdf_joint_name << ":" << std::endl;
                        // std::cout << "q_min = " << q_min << std::endl;
                        // std::cout << "q_max = " << q_max << std::endl;
                        // std::cout << "qd_max = " << qd_max << std::endl; 
                        // std::cout << "tau_max = " << tau_max << std::endl;
                        // std::cout << "\n";
                    }
                }

            }
        }
    }

    Eigen::Vector3d Aliengo::getFramePosition(const robotlib::JointState &q,
                                         const std::string origin,
                                         const std::string destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model_pin, robot_data_pin, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();
        
        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);
        
        auto origin_frame_idx = robot_model_pin.getFrameId(origin);
        auto dest_frame_idx = robot_model_pin.getFrameId(destination);
        auto oMorigin = robot_data_pin.oMf[origin_frame_idx];
        auto oMdest = robot_data_pin.oMf[dest_frame_idx];

        return (oMdest.inverse() * oMorigin).translation();
    }
    
    Eigen::Matrix3d Aliengo::getFrameOrientation(const robotlib::JointState &q,
                                        const std::string origin,
                                        const std::string destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model_pin, robot_data_pin, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();
        
        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);
        
        auto origin_frame_idx = robot_model_pin.getFrameId(origin);
        auto dest_frame_idx = robot_model_pin.getFrameId(destination);
        auto oMorigin = robot_data_pin.oMf[origin_frame_idx];
        auto oMdest = robot_data_pin.oMf[dest_frame_idx];

        return (oMdest.inverse() * oMorigin).rotation();
    }

    Eigen::Matrix4d Aliengo::getFramePose(const robotlib::JointState &q,
                                    const std::string origin,
                                    const std::string destination)
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }
    
    void Aliengo::computeLimbsJacobian(    const robotlib::JointState &q,
                                            const std::string& frame_name,
                                            Eigen::MatrixXd &jacobian){
            // map robotlib to pinocchio
            Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);

            // compute jacobian in base frame: since we are setting the robot pose to Identity (using fromRobotlibToPinocchioJointState(q)) the jacobian computed in LOCAL_WORLD_ALIGNED is the jacobian in the base frame
            const int frame_id = robot_model_pin.getFrameId(frame_name);
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model_pin.nv);
            pinocchio::computeFrameJacobian(robot_model_pin, robot_data_pin, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

            // set output
            const int jacobian_cols = robot_model_pin.nv;-6; // -6 because we are not considering the floating base joint
            jacobian = J.block(0,6,6, jacobian_cols); //6 because it is a geometric jacobian (lin, ang)

            jacobian = reorderLimbsJacobian(jacobian);
    }

    void Aliengo::computeWholeBodyJacobian( 
                                            const Eigen::Matrix<double, 7, 1> &robot_pose,   
                                            const robotlib::JointState &q,
                                            const std::string& frame_name,
                                            Eigen::MatrixXd &jacobian){
        // map robotlib to pinocchio
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(robot_pose, q);

        // compute jacobian in world frame        
        const int frame_id = robot_model_pin.getFrameId(frame_name);
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model_pin.nv);
        pinocchio::computeFrameJacobian(robot_model_pin, robot_data_pin, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // compute jacobian in base frame
        Eigen::Matrix3d b_R_w = robotlib::utils::quatToRotMat(Eigen::Quaterniond(q_pin.block<4,1>(3,0))); // orientation of the world frame expressed in base frame
        jacobian.block(0,0,3, robot_model_pin.nv) = b_R_w * J.block(0,0,3, robot_model_pin.nv);
        jacobian.block(3,0,3, robot_model_pin.nv) = b_R_w * J.block(3,0,3, robot_model_pin.nv);

        jacobian = reorderWholeBodyJacobian(jacobian);
    }


    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model_pin);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position.vec_());
        return q;
    }
    
    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const Eigen::Matrix<double, 7, 1> &robot_pose, const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model_pin);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position.vec_());
        q.head(7) = robot_pose;
        return q;
    }

    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model_pin.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity.vec_());
        return qd;
    }
    
    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model_pin.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity.vec_());
        qd.head(6) = robot_velocity;
        return qd;
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);

        pinocchio::forwardKinematics(robot_model_pin, robot_data_pin, q);
        pinocchio::updateFramePlacements(robot_model_pin, robot_data_pin);

        pinocchio::FrameIndex base_frame_id = robot_model_pin.getFrameId(robot_model_pin.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data_pin.oMf[base_frame_id].inverse();

        // feet positions
        end_effector_position["LF"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("lf_foot")]).translation();
        end_effector_position["RF"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("rf_foot")]).translation();
        end_effector_position["LH"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("lh_foot")]).translation();
        end_effector_position["RH"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("rh_foot")]).translation();
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(joint_velocity);

        pinocchio::forwardKinematics(robot_model_pin, robot_data_pin, q, qd);
        pinocchio::updateFramePlacements(robot_model_pin, robot_data_pin);

        pinocchio::FrameIndex base_frame_id = robot_model_pin.getFrameId(robot_model_pin.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data_pin.oMf[base_frame_id].inverse();

        // feet positions
        end_effector_position["LF"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("lf_foot")]).translation();
        end_effector_position["RF"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("rf_foot")]).translation();
        end_effector_position["LH"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("lh_foot")]).translation();
        end_effector_position["RH"] = (baseMo*robot_data_pin.oMf[robot_model_pin.getFrameId("rh_foot")]).translation();

        // feet velocities
        end_effector_velocity["LF"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model_pin, robot_data_pin, robot_model_pin.getFrameId("lf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["RF"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model_pin, robot_data_pin, robot_model_pin.getFrameId("rf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["LH"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model_pin, robot_data_pin, robot_model_pin.getFrameId("lh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["RH"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model_pin, robot_data_pin, robot_model_pin.getFrameId("rh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    }

    pinocchio::FrameIndex Aliengo::getBaseID() const{
        return robot_model_pin.getFrameId(robot_model_pin.frames[2].name);
    }

    Eigen::VectorXd Aliengo::clik(const std::string &frame_name,
                        const Eigen::VectorXd &q_init_guess,
                        const pinocchio::SE3 &oMdes,
                        const IK::TASK task_type)
    {
        // task dimension
        int task_dim = 0;
        if (task_type == IK::POSITION_TASK)
            task_dim = 3;
        else if (task_type == IK::POSE_TASK)
            task_dim = 6;
        else
        {
            std::cout << "Error: task type not recognized." << std::endl;
            return q_init_guess;
        }

        // IDs
        pinocchio::FrameIndex frame_id = robot_model_pin.getFrameId(frame_name);
        pinocchio::FrameIndex base_frame_id = getBaseID();

        // jacobians
		pinocchio::Data::Matrix6x J = Eigen::MatrixXd::Zero(6, robot_model_pin.nv);
        Eigen::MatrixXd J_task= Eigen::MatrixXd::Zero(task_dim, robot_model_pin.nv);
        Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
        Eigen::MatrixXd J_task_pseudo = Eigen::MatrixXd::Zero(J_task.cols(), J_task.rows());
		Eigen::VectorXd qd_des = Eigen::VectorXd::Zero(robot_model_pin.nv);

        // task error and error related variables
        Eigen::Vector<double, Eigen::Dynamic> err_task = Eigen::Vector<double, Eigen::Dynamic>::Zero(task_dim);
        Eigen::Matrix3d b_R_o = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d b_R_f = Eigen::Matrix3d::Identity();
        pinocchio::SE3 fMd = pinocchio::SE3::Identity();

        // initial guess
        Eigen::VectorXd q_des = q_init_guess;

        // CLIK parameters
		bool success = false;
        const double err_threshold = 1e-3;
		const int max_iterations = 100;
		const double dt = 0.5;
		const double damp = 1e-6;

		for (int i = 0;i<max_iterations; i++)
		{
            // compute error in base frame
            // -- compute frame placement
            pinocchio::framesForwardKinematics(robot_model_pin, robot_data_pin, q_des);

            b_R_o = robot_data_pin.oMf[base_frame_id].inverse().rotation();
            // -- compute error
            if (task_type == IK::TASK::POSITION_TASK){
                err_task = b_R_o*(oMdes.translation()) - b_R_o*(robot_data_pin.oMf[frame_id].translation()-robot_data_pin.oMf[base_frame_id].translation());
            }
            else if (task_type == IK::TASK::POSE_TASK){
			    fMd = robot_data_pin.oMf[frame_id].actInv(oMdes);
                err_task = pinocchio::log6(fMd).toVector();
                // -- rotate error in base frame
                b_R_f = b_R_o*robot_data_pin.oMf[frame_id].rotation();
                err_task.head(3) = b_R_f*err_task.head(3);
                err_task.tail(3) = b_R_f*err_task.tail(3);
            }

            if (err_task.norm() < err_threshold)
			{
                success = true;
                break;
			}

            // CLIK method (as in Handbook of Robotics eq. 10.29)

            // -- define task jacobian
			// --- compute frame jacobian in base frame
            pinocchio::computeFrameJacobian(robot_model_pin, robot_data_pin, q_des, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
            // --- compute task jacobian
            J_task = J.block(0,0,task_dim,robot_model_pin.nv);
            J_task.block(0,0,task_dim,6).setZero(); //the derivative of the task is the derivative of the error w.r.t. base frame, so the jbase jacobian is not needed
            // --- compute pseudo-inverse
            JJt_task = J_task*J_task.transpose() + Eigen::MatrixXd::Identity(JJt_task.rows(),JJt_task.cols())*damp;
            J_task_pseudo = J_task.transpose()*(JJt_task.inverse());
            
            // -- compute new joint position
            qd_des = J_task_pseudo * err_task;
            // -- fixed base inverse kinematics
            q_des.tail(this->getNJOINTS()) = q_des.tail(this->getNJOINTS()) + (qd_des).tail(this->getNJOINTS())*dt;
		}

		if (success)
		{
            return q_des;
		}
		else
		{
			std::cout
			<< "\nError: CLIK method for IK did not converge after " << max_iterations << " iterations. Error norm: "<<err_task.norm()<<", error norm threshold: " << err_threshold<<". Returning initial guess."<<std::endl;
            return q_init_guess;
		}
    }

    void Aliengo::fixedBaseInverseKinematics(
        const std::string &frame_name,
        const robotlib::JointState &q_init_guess,
        const Eigen::Vector3d &position_des,
        robotlib::JointState &q_des)
    {
        // compute desired pose in pinocchio format
		const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), position_des);
        // map from robotlib to pinocchio
        Eigen::VectorXd q_pin_init_guess = fromRobotlibToPinocchioJointState(q_init_guess);

        // Closed Loop Inverse Kinematics (CLIK)
        Eigen::VectorXd q_pin_des = clik(frame_name, q_pin_init_guess, oMdes, IK::TASK::POSITION_TASK);
       
        // Get IK solution
        q_des = reorderJoints(q_pin_des.tail(this->getNJOINTS()));
    }

    void Aliengo::fixedBaseInverseDiffKinematics(const std::string &frame_name,
                                        const robotlib::JointState &q,
                                        const Eigen::Vector3d &velocity_des,
                                        robotlib::JointState &qd_des)
    {    
        // map from robotlib to pinocchio
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        // compute jacobian in base frame
        pinocchio::FrameIndex frame_id = robot_model_pin.getFrameId(frame_name);
		pinocchio::Data::Matrix6x J = Eigen::MatrixXd::Zero(6, robot_model_pin.nv);
        pinocchio::framesForwardKinematics(robot_model_pin, robot_data_pin, q_pin);
        pinocchio::computeFrameJacobian(robot_model_pin, robot_data_pin, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // compute pseudo-inverse using linear jacobian        
        Eigen::MatrixXd J_lin = J.block(0,6,3,robot_model_pin.nv-6); // do not consider base
		const double damp = 1e-6;
        Eigen::MatrixXd JJt_lin = Eigen::MatrixXd::Identity(J_lin.rows(),J_lin.rows())*damp;
        JJt_lin += J_lin*J_lin.transpose();
        Eigen::MatrixXd J_pseudo = J_lin.transpose()*(JJt_lin.inverse());
        // compute desired joint velocity
        qd_des = J_pseudo * velocity_des;                       
    }

    void Aliengo::inverseDynamics(const Eigen::Matrix<double, 7, 1> &robot_pose,    // robot base
                                const Eigen::Matrix<double, 6, 1> &robot_velocity,    // robot base
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,  // robot base
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                const robotlib::JointState &joint_acceleration,
                                const robotlib::eigen::aligned_map<std::string, Eigen::Vector3d> &f_contact,
                                robotlib::JointState &tau_joints)             ///output
    {
        // map from robotlib to pinocchio
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(robot_pose, joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(robot_velocity, joint_velocity);
        Eigen::VectorXd qdd = fromRobotlibToPinocchioJointVelocity(robot_acceleration, joint_acceleration);

        // compute contact forces in joint frame
        pinocchio::container::aligned_vector<pinocchio::Force> joint_f_contact(robot_model_pin.njoints, pinocchio::Force::Zero());
        for(auto &[frame_name, force] : f_contact)
        {
            const pinocchio::FrameIndex frame_id = robot_model_pin.getFrameId(frame_name);
            const pinocchio::JointIndex joint_id = robot_model_pin.frames[frame_id].parentJoint;
            pinocchio::Force pin_force(force, Eigen::Vector3d::Zero());
			joint_f_contact[joint_id] = robot_data_pin.oMi[joint_id].actInv(
											robot_data_pin.oMf[frame_id].act(pin_force));
        }
        pinocchio::rnea(robot_model_pin, robot_data_pin, q, qd, qdd, joint_f_contact);
        tau_joints = reorderJoints(robot_data_pin.tau.tail(this->getNJOINTS()));
    }


    void Aliengo::computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          Eigen::Matrix<double, 6, 1> &g_base,
                                          robotlib::JointState &g_joints)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(robot_pose, joint_position);

        pinocchio::computeGeneralizedGravity(robot_model_pin, robot_data_pin, q); // equivalent to pinocchio::rnea(model, data, q, 0, 0).

		g_base = robot_data_pin.g.block<6,1>(0,0);
		g_joints = reorderJoints(robot_data_pin.g.tail(this->getNJOINTS()));
    }
    
    void Aliengo::computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          robotlib::JointState &g_joints)
    {
        Eigen::Matrix<double, 6, 1> g_base = Eigen::Matrix<double, 6, 1>::Zero();
        computeGravityTerm(robot_pose, joint_position, g_base, g_joints);
    }

    void Aliengo::computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        Eigen::Matrix<double, 6, 1> &nle_base,
                                        robotlib::JointState &nle_joints
                                        )
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(robot_pose, joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(robot_velocity, joint_velocity);
        pinocchio::nonLinearEffects(robot_model_pin, robot_data_pin, q, qd);

		nle_base = robot_data_pin.nle.block<6,1>(0,0);
		nle_joints = reorderJoints(robot_data_pin.nle.tail(this->getNJOINTS()));
    }

    void Aliengo::computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        robotlib::JointState &nle_joints)
    {
        Eigen::Matrix<double, 6, 1> nle_base = Eigen::Matrix<double, 6, 1>::Zero();
        computeNonLinearEffects(robot_pose, Eigen::Matrix<double,6,1>::Zero(), joint_position, joint_velocity, nle_base, nle_joints);
    }

     void Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                    robotlib::JointState &joint_position)
     {
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        robcogen_joint_position.setZero();

        for (auto leg : *legs_)
        {
            const int leg_id{glue_leg_names_to_ids.at(leg->getName())};

            robcogen_end_effector_position[leg_id] = end_effector_position[leg];
        }

        inverse_kinematics_->getJointPosition(robcogen_end_effector_position, robcogen_joint_position);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            joint_position[joint] = robcogen_joint_position[joint_id];
        }
    }

    void Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                       const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                       robotlib::JointState &joint_position,
                                       robotlib::JointState &joint_velocity)
    {
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_position{};
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_velocity{};
        
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_velocity{};

        robcogen_joint_position.setZero();
        robcogen_joint_velocity.setZero();


        for (auto leg : *legs_)
        {
            const int leg_id{glue_leg_names_to_ids.at(leg->getName())};

            robcogen_end_effector_position[leg_id] = end_effector_position[leg];
            robcogen_end_effector_velocity[leg_id] = end_effector_velocity[leg];            
        }

        inverse_kinematics_->getJointState(robcogen_end_effector_position,
                                           robcogen_end_effector_velocity,
                                           robcogen_joint_position,
                                           robcogen_joint_velocity);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            joint_position[joint] = robcogen_joint_position[joint_id];
            joint_velocity[joint] = robcogen_joint_velocity[joint_id];
        }
    }

    void Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity,
                                    const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                    robotlib::JointState &joint_position,
                                    robotlib::JointState &joint_velocity,
                                    robotlib::JointState &joint_acceleration)
    {
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_position{};
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_velocity{};
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_acceleration{};
        
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_velocity{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_acceleration{};

        robcogen_joint_position.setZero();
        robcogen_joint_velocity.setZero();
        robcogen_joint_acceleration.setZero();

        for (auto leg : *legs_)
        {
            const int leg_id{glue_leg_names_to_ids.at(leg->getName())};

            robcogen_end_effector_position[leg_id] = end_effector_position[leg];
            robcogen_end_effector_velocity[leg_id] = end_effector_velocity[leg];
            robcogen_end_effector_acceleration[leg_id] = end_effector_acceleration[leg];
            
        }

        inverse_kinematics_->getJointState(robcogen_end_effector_position,
                                           robcogen_end_effector_velocity,
                                           robcogen_end_effector_acceleration,
                                           robcogen_joint_position,
                                           robcogen_joint_velocity,
                                           robcogen_joint_acceleration);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            joint_position[joint] = robcogen_joint_position[joint_id];
            joint_velocity[joint] = robcogen_joint_velocity[joint_id];
            joint_acceleration[joint] = robcogen_joint_acceleration[joint_id];
        }
    }

    Eigen::Matrix<double, 3, 1> Aliengo::getWholeBodyCOM(const robotlib::JointState &joint_position) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> joint_position_matrix = Eigen::Matrix<double, NJOINTS_TOT, 1>::Zero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            joint_position_matrix[joint_id] = joint_position[joint];
        }
        // First updates the coordinate transforms that will be used by the routine
        
        homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg(joint_position_matrix);

        // The actual calculus
        Eigen::Matrix<double, 3, 1> tmpSum = Eigen::Matrix<double, 3, 1>::Zero();

        tmpSum += inertias_->getCOM_trunk() * inertias_->getMass_trunk();

        iit::Aliengo::HomogeneousTransforms::MatrixType tmpX(iit::Aliengo::HomogeneousTransforms::MatrixType::Identity());
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_lf_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_rf_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_lh_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_rh_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_lc_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_rc_haa_chain;

        base_X_lf_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly;
        tmpSum += inertias_->getMass_LF_hipassembly() *
                  (iit::rbd::Utils::transform(base_X_lf_haa_chain, inertias_->getCOM_LF_hipassembly()));

        base_X_lf_haa_chain = base_X_lf_haa_chain * homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg;
        tmpSum += inertias_->getMass_LF_upperleg() *
                  (iit::rbd::Utils::transform(base_X_lf_haa_chain, inertias_->getCOM_LF_upperleg()));

        base_X_lf_haa_chain = base_X_lf_haa_chain * homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg;
        tmpSum += inertias_->getMass_LF_lowerleg() *
                  (iit::rbd::Utils::transform(base_X_lf_haa_chain, inertias_->getCOM_LF_lowerleg()));

        base_X_rf_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly;
        tmpSum += inertias_->getMass_RF_hipassembly() *
                  (iit::rbd::Utils::transform(base_X_rf_haa_chain, inertias_->getCOM_RF_hipassembly()));

        base_X_rf_haa_chain = base_X_rf_haa_chain * homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg;
        tmpSum += inertias_->getMass_RF_upperleg() *
                  (iit::rbd::Utils::transform(base_X_rf_haa_chain, inertias_->getCOM_RF_upperleg()));

        base_X_rf_haa_chain = base_X_rf_haa_chain * homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg;
        tmpSum += inertias_->getMass_RF_lowerleg() *
                  (iit::rbd::Utils::transform(base_X_rf_haa_chain, inertias_->getCOM_RF_lowerleg()));

        base_X_lh_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly;
        tmpSum += inertias_->getMass_LH_hipassembly() *
                  (iit::rbd::Utils::transform(base_X_lh_haa_chain, inertias_->getCOM_LH_hipassembly()));

        base_X_lh_haa_chain = base_X_lh_haa_chain * homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg;
        tmpSum += inertias_->getMass_LH_upperleg() *
                  (iit::rbd::Utils::transform(base_X_lh_haa_chain, inertias_->getCOM_LH_upperleg()));

        base_X_lh_haa_chain = base_X_lh_haa_chain * homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg;
        tmpSum += inertias_->getMass_LH_lowerleg() *
                  (iit::rbd::Utils::transform(base_X_lh_haa_chain, inertias_->getCOM_LH_lowerleg()));

        base_X_rh_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly;
        tmpSum += inertias_->getMass_RH_hipassembly() *
                  (iit::rbd::Utils::transform(base_X_rh_haa_chain, inertias_->getCOM_RH_hipassembly()));

        base_X_rh_haa_chain = base_X_rh_haa_chain * homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg;
        tmpSum += inertias_->getMass_RH_upperleg() *
                  (iit::rbd::Utils::transform(base_X_rh_haa_chain, inertias_->getCOM_RH_upperleg()));

        base_X_rh_haa_chain = base_X_rh_haa_chain * homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg;
        tmpSum += inertias_->getMass_RH_lowerleg() *
                  (iit::rbd::Utils::transform(base_X_rh_haa_chain, inertias_->getCOM_RH_lowerleg()));

        return tmpSum / inertias_->getTotalMass();
    }

    Eigen::Vector3d Aliengo::getCoMFromBase(const robotlib::JointState &q,
                                const Eigen::Vector3d &base_orient,
                                const Eigen::Vector3d &base_pos) const
    {
        Eigen::Matrix3d R = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = getWholeBodyCOM(q);
        return base_pos + R.transpose() * offCoM;         //CoM is in the world frame off CoM is in base frame
    }

    Eigen::Vector3d Aliengo::getBaseFromCoM(const robotlib::JointState &q,
                                const Eigen::Vector3d &base_orient,
                                const Eigen::Vector3d &CoM) const
    {
        Eigen::Matrix3d b_R_w = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = getWholeBodyCOM(q);
        return CoM - b_R_w.transpose() * offCoM;          //CoM is in the world frame off CoM is in base frame
    }

    //compute spatial velocity of the CoM (in base frame) (only joint influence)
    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVel(const robotlib::JointState &q,
                                                            const robotlib::JointState &qd) const
    {
        q.size();   // TODO: Not used. Created to remove warning
        qd.size();   // TODO: Not used. Created to remove warning

        Eigen::Matrix<double, 6, 1> CoMvel = Eigen::Matrix<double, 6, 1>::Zero();
        
        std::cout << "TODO: getWholeBodyCOMVel\n";

        // The actual calculus
        //TODO1
        //CoMvel = Crex::rcg::getWholeBodyCOMJacobian(q, inertiaProps, ht)*qd;
        return CoMvel;
    }
    
    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const robotlib::JointState &q) const
    {
        Eigen::Matrix<double, 6, 1> CoMVel = Eigen::Matrix<double, 6, 1>::Zero();

        CoMVel = iit::motionVectorTransform(getWholeBodyCOM(q), R) * baseVel;

        return CoMVel;
    }

    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                            const Eigen::Matrix3d &R,
                                                            const Eigen::Vector3d offset_com) const
    {
        Eigen::Matrix<double, 6, 1> CoMVel = Eigen::Matrix<double, 6, 1>::Zero();

        CoMVel = iit::motionVectorTransform(offset_com, R) * baseVel;

        return CoMVel;
    }

    Eigen::Vector3d Aliengo::getLegContribution(const robotlib::JointState &q) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> joint_position_matrix = Eigen::Matrix<double, NJOINTS_TOT, 1>::Zero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            joint_position_matrix[joint_id] = q[joint];
        }

        // First updates the coordinate transforms that will be used by the routine
        homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly(joint_position_matrix);
        homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg(joint_position_matrix);
        homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg(joint_position_matrix);
        homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg(joint_position_matrix);

         Eigen::Vector3d tmpSum = Eigen::Vector3d::Zero();

        iit::Aliengo::HomogeneousTransforms::MatrixType tmpX(iit::Aliengo::HomogeneousTransforms::MatrixType::Identity());
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_LF_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_RF_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_LH_haa_chain;
        iit::Aliengo::HomogeneousTransforms::MatrixType base_X_RH_haa_chain;

        base_X_LF_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly;
        tmpSum += inertias_->getMass_LF_hipassembly() *
                (iit::rbd::Utils::transform(base_X_LF_haa_chain, inertias_->getCOM_LF_hipassembly()));

        base_X_LF_haa_chain = base_X_LF_haa_chain * homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg;
        tmpSum += inertias_->getMass_LF_upperleg() *
                (iit::rbd::Utils::transform(base_X_LF_haa_chain, inertias_->getCOM_LF_upperleg()));

        base_X_LF_haa_chain = base_X_LF_haa_chain * homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg;
        tmpSum += inertias_->getMass_LF_lowerleg() *
                (iit::rbd::Utils::transform(base_X_LF_haa_chain, inertias_->getCOM_LF_lowerleg()));

        base_X_RF_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly;
        tmpSum += inertias_->getMass_RF_hipassembly() *
                (iit::rbd::Utils::transform(base_X_RF_haa_chain, inertias_->getCOM_RF_hipassembly()));

        base_X_RF_haa_chain = base_X_RF_haa_chain * homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg;
        tmpSum += inertias_->getMass_RF_upperleg() *
                (iit::rbd::Utils::transform(base_X_RF_haa_chain, inertias_->getCOM_RF_upperleg()));

        base_X_RF_haa_chain = base_X_RF_haa_chain * homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg;
        tmpSum += inertias_->getMass_RF_lowerleg() *
                (iit::rbd::Utils::transform(base_X_RF_haa_chain, inertias_->getCOM_RF_lowerleg()));

        base_X_LH_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly;
        tmpSum += inertias_->getMass_LH_hipassembly() *
                (iit::rbd::Utils::transform(base_X_LH_haa_chain, inertias_->getCOM_LH_hipassembly()));

        base_X_LH_haa_chain = base_X_LH_haa_chain * homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg;
        tmpSum += inertias_->getMass_LH_upperleg() *
                (iit::rbd::Utils::transform(base_X_LH_haa_chain, inertias_->getCOM_LH_upperleg()));

        base_X_LH_haa_chain = base_X_LH_haa_chain * homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg;
        tmpSum += inertias_->getMass_LH_lowerleg() *
                (iit::rbd::Utils::transform(base_X_LH_haa_chain, inertias_->getCOM_LH_lowerleg()));

        base_X_RH_haa_chain = tmpX * homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly;
        tmpSum += inertias_->getMass_RH_hipassembly() *
                (iit::rbd::Utils::transform(base_X_RH_haa_chain, inertias_->getCOM_RH_hipassembly()));

        base_X_RH_haa_chain = base_X_RH_haa_chain * homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg;
        tmpSum += inertias_->getMass_RH_upperleg() *
                (iit::rbd::Utils::transform(base_X_RH_haa_chain, inertias_->getCOM_RH_upperleg()));

        base_X_RH_haa_chain = base_X_RH_haa_chain * homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg;
        tmpSum += inertias_->getMass_RH_lowerleg() *
                (iit::rbd::Utils::transform(base_X_RH_haa_chain, inertias_->getCOM_RH_lowerleg()));

        return tmpSum / (getRobotMass() - getTrunkMass());
    }

    Eigen::Matrix4d Aliengo::getImuBaseOffset(const std::string& imu_link_name, const std::string& base_link_name) const
    {
        //Imu pose in base frame updated recursively
        Eigen::Matrix4d imu_pose {Eigen::Matrix4d::Zero()};
        imu_pose(3,3) = 1; //homogeneous definition
        bool reached_base_frame{imu_link_name==base_link_name};
        if(!reached_base_frame)
        {
            // Get parent joint of imu link
            std::shared_ptr<urdf::Joint> current_joint = robot_model_.getLink(imu_link_name)->parent_joint;

            // Initialize imu_pose before iterating
            urdf::Pose current_urdf_pose (current_joint->parent_to_joint_origin_transform);
            imu_pose = robotlib::utils::get_eigen_matrix4d_from_urdf_pose(current_urdf_pose);

            // Backward recursion from link frame to which the IMU is attached (actually in the urdf the link frame is the frame of the parent joint of the link) to base frame
            while(!reached_base_frame)
            {
                current_joint = robot_model_.getLink(current_joint->parent_link_name)->parent_joint;
                urdf::Pose current_urdf_pose (current_joint->parent_to_joint_origin_transform);
                Eigen::Matrix4d current_pose {robotlib::utils::get_eigen_matrix4d_from_urdf_pose(current_urdf_pose)};
                imu_pose.block<3,3>(0,0) = current_pose.block<3,3>(0,0) * imu_pose.block<3,3>(0,0);
                imu_pose.block<3,1>(0,3) = current_pose.block<3,1>(0,3) + current_pose.block<3,3>(0,0) * imu_pose.block<3,1>(0,3);
                reached_base_frame = current_joint->parent_link_name == base_link_name;
            }
        }
        return imu_pose;
    }
    
    double Aliengo::getTrunkMass() const
    {
        return inertias_->getTrunkMass();
    }

    double Aliengo::getLegsMass() const
    {
        return inertias_->getLegMass();
    }
    
    void Aliengo::setInvKinTimePeriod(const double period)
    {
        inverse_kinematics_->setTimePeriod(period);
    } 

    void Aliengo::setTrunkCom(const Eigen::Vector3d &trunk_com)
    {
        robot_params_->setValue_trunk_com_x(trunk_com(0));
        robot_params_->setValue_trunk_com_y(trunk_com(1));
        robot_params_->setValue_trunk_com_z(trunk_com(2));
    }

    void Aliengo::setTrunkMass(const double trunk_mass)
    {
        robot_params_->setValue_trunk_mass(trunk_mass);   
    }

    double Aliengo::getRobotMass() const
    {
        return inertias_->getTotalMass(); // pinocchio::computeTotalMass(robot_model_pin, robot_data_pin);
    }

    Eigen::Vector3d Aliengo::getRobotCoM() const {
        std::cout << "TODO: getRobotCoM" << std::endl;
        return Eigen::Vector3d().setZero();
    }

    Eigen::Matrix<double, 3, 1> Aliengo::getTrunkCOM() const
    {
        return inertias_->getCOM_trunk();
    }

    Eigen::VectorXd Aliengo::reorderJoints(const Eigen::VectorXd& data) const{
        Eigen::VectorXd new_data = data;
        for(auto &[key, value] : idx_map)
        {
            new_data[value] = data[key];
            new_data[key] = data[value];
        }
        return new_data;
    }

    Eigen::MatrixXd Aliengo::reorderLimbsJacobian(const Eigen::MatrixXd& jacobian) const{
        Eigen::MatrixXd new_jac = jacobian;
        for(auto &[key, value] : idx_map)
        {
            new_jac.block<6,1>(0,value) = jacobian.block<6,1>(0,key);
            new_jac.block<6,1>(0,key) = jacobian.block<6,1>(0,value);
        }
        return new_jac;
    }

    Eigen::MatrixXd Aliengo::reorderWholeBodyJacobian(const Eigen::MatrixXd& jacobian) const{
        Eigen::MatrixXd new_jac = jacobian;
        for(auto &[key, value] : idx_map)
        {
            new_jac.block<6,1>(0,value+6) = jacobian.block<6,1>(0,key+6);
            new_jac.block<6,1>(0,key+6) = jacobian.block<6,1>(0,value+6);
        }
        return new_jac;
    }

    std::shared_ptr<AliengoLeg> makeLeg(const std::string &legName) // function used to generate a leg inside the create_function
    {
        std::shared_ptr<robotlib::Joint> haa = std::make_shared<robotlib::Joint>(legName + "_HAA");
        std::shared_ptr<robotlib::Link> assembly = std::make_shared<robotlib::Link>(legName + "_ASSEMBLY");
        std::shared_ptr<robotlib::Joint> hfe = std::make_shared<robotlib::Joint>(legName + "_HFE");

        std::shared_ptr<robotlib::Link> upperleg = std::make_shared<robotlib::Link>(legName + "_UPPERLEG");
        std::shared_ptr<robotlib::Joint> kfe = std::make_shared<robotlib::Joint>(legName + "_KFE");
        std::shared_ptr<robotlib::Link> lowerleg = std::make_shared<robotlib::Link>(legName + "_LOWERLEG");

        return std::make_shared<AliengoLeg>(legName,
                                            std::array<std::shared_ptr<robotlib::Joint>, NJOINTS>({haa, hfe, kfe}),
                                            std::array<std::shared_ptr<robotlib::Link>, NLINKS>({assembly, upperleg, lowerleg}));
    }

    extern "C" std::shared_ptr<robotlib::RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf)
    {
        const std::shared_ptr<robotlib::Trunk> trunk = std::make_shared<robotlib::Trunk>("TRUNK");
        const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs(
            {makeLeg("LF"),
             makeLeg("RF"),
             makeLeg("LH"),
             makeLeg("RH")});
        const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms({});

        return std::make_shared<Aliengo>(trunk, legs, arms, robot_urdf);
    }
    
    extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
    {
        std::string urdf_path("/usr/include/aliengo_description/urdfs/aliengo.urdf");

        if (!std::filesystem::exists(urdf_path))
        {
            const std::string error{urdf_path + " not found"};
            throw error;
        }

        std::ifstream myfile{urdf_path};
        std::stringstream ss;
        ss << myfile.rdbuf();
        std::string robot_description{ss.str()};

        return createRobotWithUrdf_t(robot_description);
    }

    extern "C" void destroyRobotWithUrdf_t(std::shared_ptr<robotlib::RobotBase> robot)
    {
    }

    extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase>)
    {
    }

} // namespace aliengolib