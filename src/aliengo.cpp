#include "aliengolib/aliengo.hpp"
#include "aliengolib/aliengo_leg.hpp"

#include "robotlib/utils/utils.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"
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
        pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), robot_model);
        robot_data = pinocchio::Data(robot_model);

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

        setJointLimitsFromUrdf();
    }

    Aliengo::~Aliengo(){}

    void Aliengo::setJointLimitsFromUrdf()
    {
        for(auto leg : *this->legs_)
        {
            for(auto joint: *leg->getJoints()){
                // **Transform robotlib joint name to urdf one**
                std::string urdf_joint_name{joint->getName()};
                std::transform(urdf_joint_name.begin(), urdf_joint_name.end(), urdf_joint_name.begin(), ::tolower);
                urdf_joint_name.append("_joint");

                const int joint_id_nq = getJointIdForNq(urdf_joint_name);
                const int joint_id_nv = getJointIdForNv(urdf_joint_name);
                const double q_min = robot_model.lowerPositionLimit[joint_id_nq];
                const double q_max = robot_model.upperPositionLimit[joint_id_nq];
                const double qd_max = robot_model.velocityLimit[joint_id_nv];
                const double tau_max = robot_model.effortLimit[joint_id_nv];
                setJointLimits(joint, q_min, q_max, qd_max, tau_max);
            }
        }
    }

    int Aliengo::getJointIdForNq(const std::string &joint_name) const
    {
        return robot_model.getJointId(joint_name)+5;//5: in any variable of dimension nq, with a model having a JointModelFreeFlyer as root_joint, the first 7 values corresponds to the floating base joint (pos,quaternion). But the number of joints are universe+root_joint+robot_joints. So the index of the first robot joint is 2, which corresponds to 7 in any variable of dimension nq. So we use the joint_id offset=5.
    }

    int Aliengo::getJointIdForNv(const std::string &joint_name) const
    {
        return robot_model.getJointId(joint_name)+4;//4: in any variable of dimension nv, with a model having a JointModelFreeFlyer as root_joint, the first 6 values corresponds to the floating base joint (pos,quaternion). But the number of joints are universe+root_joint+robot_joints. So the index of the first robot joint is 2, which corresponds to 6 in any variable of dimension nq. So we use the joint_id offset=4.
    }

    Eigen::Vector3d Aliengo::getFramePosition(const robotlib::JointState &q,
                                         const std::string origin,
                                         const std::string destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();
        
        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);
        
        auto origin_frame_idx = robot_model.getFrameId(origin);
        auto dest_frame_idx = robot_model.getFrameId(destination);
        auto oMorigin = robot_data.oMf[origin_frame_idx];
        auto oMdest = robot_data.oMf[dest_frame_idx];

        return (oMdest.inverse() * oMorigin).translation();
    }
    
    Eigen::Matrix3d Aliengo::getFrameOrientation(const robotlib::JointState &q,
                                        const std::string origin,
                                        const std::string destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();
        
        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);
        
        auto origin_frame_idx = robot_model.getFrameId(origin);
        auto dest_frame_idx = robot_model.getFrameId(destination);
        auto oMorigin = robot_data.oMf[origin_frame_idx];
        auto oMdest = robot_data.oMf[dest_frame_idx];

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
            const int frame_id = robot_model.getFrameId(frame_name);
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
            pinocchio::computeFrameJacobian(robot_model, robot_data, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

            // set output
            const int jacobian_cols = robot_model.nv-6; // -6 because we are not considering the floating base joint
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
        const int frame_id = robot_model.getFrameId(frame_name);
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
        pinocchio::computeFrameJacobian(robot_model, robot_data, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // compute jacobian in base frame
        Eigen::Matrix3d b_R_w = robotlib::utils::quatToRotMat(Eigen::Quaterniond(q_pin.block<4,1>(3,0))); // orientation of the world frame expressed in base frame
        jacobian.block(0,0,3, robot_model.nv) = b_R_w * J.block(0,0,3, robot_model.nv);
        jacobian.block(3,0,3, robot_model.nv) = b_R_w * J.block(3,0,3, robot_model.nv);

        jacobian = reorderWholeBodyJacobian(jacobian);
    }


    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position.vec_());
        return q;
    }
    
    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const Eigen::Matrix<double, 7, 1> &robot_pose, const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position.vec_());
        q.head(7) = robot_pose;
        return q;
    }

    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity.vec_());
        return qd;
    }
    
    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity.vec_());
        qd.head(6) = robot_velocity;
        return qd;
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);

        pinocchio::forwardKinematics(robot_model, robot_data, q);
        pinocchio::updateFramePlacements(robot_model, robot_data);

        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId(robot_model.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data.oMf[base_frame_id].inverse();

        // feet positions
        end_effector_position["LF"] = (baseMo*robot_data.oMf[robot_model.getFrameId("lf_foot")]).translation();
        end_effector_position["RF"] = (baseMo*robot_data.oMf[robot_model.getFrameId("rf_foot")]).translation();
        end_effector_position["LH"] = (baseMo*robot_data.oMf[robot_model.getFrameId("lh_foot")]).translation();
        end_effector_position["RH"] = (baseMo*robot_data.oMf[robot_model.getFrameId("rh_foot")]).translation();
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(joint_velocity);

        pinocchio::forwardKinematics(robot_model, robot_data, q, qd);
        pinocchio::updateFramePlacements(robot_model, robot_data);

        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId(robot_model.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data.oMf[base_frame_id].inverse();

        // feet positions
        end_effector_position["LF"] = (baseMo*robot_data.oMf[robot_model.getFrameId("lf_foot")]).translation();
        end_effector_position["RF"] = (baseMo*robot_data.oMf[robot_model.getFrameId("rf_foot")]).translation();
        end_effector_position["LH"] = (baseMo*robot_data.oMf[robot_model.getFrameId("lh_foot")]).translation();
        end_effector_position["RH"] = (baseMo*robot_data.oMf[robot_model.getFrameId("rh_foot")]).translation();

        // feet velocities
        end_effector_velocity["LF"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("lf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["RF"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("rf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["LH"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("lh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        end_effector_velocity["RH"] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("rh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    }

    pinocchio::FrameIndex Aliengo::getBaseID() const{
        return robot_model.getFrameId(robot_model.frames[2].name);
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
        // else if (task_type == IK::POSE_TASK)
        //     task_dim = 6;
        else
        {
            std::cout << "Error: task type not recognized." << std::endl;
            return q_init_guess;
        }

        // IDs
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(frame_name);
        pinocchio::FrameIndex base_frame_id = getBaseID();

        // jacobians
		pinocchio::Data::Matrix6x J = Eigen::MatrixXd::Zero(6, robot_model.nv);
        Eigen::MatrixXd J_task= Eigen::MatrixXd::Zero(task_dim, robot_model.nv);
        Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
        Eigen::MatrixXd J_task_pseudo = Eigen::MatrixXd::Zero(J_task.cols(), J_task.rows());
		Eigen::VectorXd qd_des = Eigen::VectorXd::Zero(robot_model.nv);

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
            pinocchio::framesForwardKinematics(robot_model, robot_data, q_des);

            b_R_o = robot_data.oMf[base_frame_id].inverse().rotation();
            // -- compute error
            if (task_type == IK::TASK::POSITION_TASK){
                err_task = b_R_o*(oMdes.translation()) - b_R_o*(robot_data.oMf[frame_id].translation()-robot_data.oMf[base_frame_id].translation());
            }
            // else if (task_type == IK::TASK::POSE_TASK){
			//     fMd = robot_data.oMf[frame_id].actInv(oMdes);
            //     err_task = pinocchio::log6(fMd).toVector();
            //     // -- rotate error in base frame
            //     b_R_f = b_R_o*robot_data.oMf[frame_id].rotation();
            //     err_task.head(3) = b_R_f*err_task.head(3);
            //     err_task.tail(3) = b_R_f*err_task.tail(3);
            // }

            if (err_task.norm() < err_threshold)
			{
                success = true;
                break;
			}

            // CLIK method (as in Handbook of Robotics eq. 10.29)

            // -- define task jacobian
			// --- compute frame jacobian in base frame
            pinocchio::computeFrameJacobian(robot_model, robot_data, q_des, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
            // --- compute task jacobian
            J_task = J.block(0,0,task_dim,robot_model.nv);
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
    
    void Aliengo::fixedBaseInverseKinematics(
        const robotlib::JointState &q_init_guess,
        const robotlib::LegDataMap<Eigen::Vector3d> &positions_des,
        robotlib::JointState &q_des)
    {
        auto q_des_temp = this->makeJointState(0.0);
        for (auto leg : *legs_)
        {
            std::string foot_name = leg->getName() + "_foot";
            foot_name = robotlib::utils::toLower(foot_name);
            this->fixedBaseInverseKinematics(foot_name, q_init_guess, positions_des[leg], q_des_temp);
            q_des[leg] = q_des_temp[leg];
        }
    }

    void Aliengo::fixedBaseInverseDiffKinematics(const std::string &frame_name,
                                        const robotlib::JointState &q,
                                        const Eigen::Vector3d &velocity_des,
                                        robotlib::JointState &qd_des)
    {    
        // map from robotlib to pinocchio
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        // compute jacobian in base frame
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(frame_name);
		pinocchio::Data::Matrix6x J = Eigen::MatrixXd::Zero(6, robot_model.nv);
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_pin);
        pinocchio::computeFrameJacobian(robot_model, robot_data, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // compute pseudo-inverse using linear jacobian        
        Eigen::MatrixXd J_lin = J.block(0,6,3,robot_model.nv-6); // do not consider base
		const double damp = 1e-6;
        Eigen::MatrixXd JJt_lin = Eigen::MatrixXd::Identity(J_lin.rows(),J_lin.rows())*damp;
        JJt_lin += J_lin*J_lin.transpose();
        Eigen::MatrixXd J_pseudo = J_lin.transpose()*(JJt_lin.inverse());
        // compute desired joint velocity
        Eigen::VectorXd qd_des_pin = J_pseudo * velocity_des;
        qd_des = reorderJoints(qd_des_pin.tail(this->getNJOINTS()));                       
    }

    void Aliengo::fixedBaseInverseDiffKinematics(const robotlib::JointState &q,
                                        const robotlib::LegDataMap<Eigen::Vector3d> &velocities_des,
                                        robotlib::JointState &qd_des)
    {
        auto qd_des_temp = this->makeJointState(0.0);
        for (auto leg : *legs_)
        {
            std::string foot_name = leg->getName() + "_foot";
            foot_name = robotlib::utils::toLower(foot_name);
            this->fixedBaseInverseDiffKinematics(foot_name, q, velocities_des[leg], qd_des_temp);
            qd_des[leg] = qd_des_temp[leg];
        }
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
        pinocchio::container::aligned_vector<pinocchio::Force> joint_f_contact(robot_model.njoints, pinocchio::Force::Zero());
        for(auto &[frame_name, force] : f_contact)
        {
            const pinocchio::FrameIndex frame_id = robot_model.getFrameId(frame_name);
            const pinocchio::JointIndex joint_id = robot_model.frames[frame_id].parentJoint;
            pinocchio::Force pin_force(force, Eigen::Vector3d::Zero());
			joint_f_contact[joint_id] = robot_data.oMi[joint_id].actInv(
											robot_data.oMf[frame_id].act(pin_force));
        }
        pinocchio::rnea(robot_model, robot_data, q, qd, qdd, joint_f_contact);
        tau_joints = reorderJoints(robot_data.tau.tail(this->getNJOINTS()));
    }


    void Aliengo::computeGravityTerm(  const Eigen::Matrix<double, 7, 1> &robot_pose,
                                          const robotlib::JointState &joint_position,
                                          Eigen::Matrix<double, 6, 1> &g_base,
                                          robotlib::JointState &g_joints)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(robot_pose, joint_position);

        pinocchio::computeGeneralizedGravity(robot_model, robot_data, q); // equivalent to pinocchio::rnea(model, data, q, 0, 0).

		g_base = robot_data.g.block<6,1>(0,0);
		g_joints = reorderJoints(robot_data.g.tail(this->getNJOINTS()));
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
        pinocchio::nonLinearEffects(robot_model, robot_data, q, qd);

		nle_base = robot_data.nle.block<6,1>(0,0);
		nle_joints = reorderJoints(robot_data.nle.tail(this->getNJOINTS()));
    }

    void Aliengo::computeNonLinearEffects( const Eigen::Matrix<double, 7, 1> &robot_pose,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        robotlib::JointState &nle_joints)
    {
        Eigen::Matrix<double, 6, 1> nle_base = Eigen::Matrix<double, 6, 1>::Zero();
        computeNonLinearEffects(robot_pose, Eigen::Matrix<double,6,1>::Zero(), joint_position, joint_velocity, nle_base, nle_joints);
    }

    Eigen::Matrix4d Aliengo::getImuBaseOffset(const std::string& imu_link_name, const std::string& base_link_name) const
    {
        Eigen::Matrix4d imu_pose {Eigen::Matrix4d::Zero()};
        imu_pose(3,3) = 1; //homogeneous definition
        int frame_id = robot_model.getFrameId(imu_link_name);
        pinocchio::Frame frame = robot_model.frames[frame_id];
        imu_pose.block<3,3>(0,0) = frame.placement.rotation();
        imu_pose.block<3,1>(0,3) = frame.placement.translation();
        std::string parent_joint_name = robot_model.names[frame.parentJoint];
        bool reached_base_frame{parent_joint_name=="root_joint"};
        while(!reached_base_frame)
        {
            frame_id = robot_model.getFrameId(parent_joint_name);
            frame = robot_model.frames[frame_id];
            imu_pose.block<3,3>(0,0) = frame.placement.rotation() * imu_pose.block<3,3>(0,0);
            imu_pose.block<3,1>(0,3) = frame.placement.translation() + frame.placement.rotation() * imu_pose.block<3,1>(0,3);
            parent_joint_name = robot_model.names[frame.parentJoint];
            reached_base_frame = parent_joint_name=="root_joint";
        }
        return imu_pose;
    }
    
    double Aliengo::getLinkMass(const std::string& name) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(name)].parentJoint;
        return robot_model.inertias[joint_id].mass();
    }

    double Aliengo::getRobotMass() const
    {
        return pinocchio::computeTotalMass(robot_model);
    }

    Eigen::Vector3d Aliengo::getWholeBodyCoM(const robotlib::JointState q) {
        // set the robot base pose to pos=0, ori=0 --> centerOfMass gives the CoM in the base frame
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q); 
        return pinocchio::centerOfMass(robot_model, robot_data, q_pin, false);//false: do not compute com of subtrees
    }

    Eigen::Vector3d Aliengo::getLinkCOM(const std::string& name) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(name)].parentJoint;
        return robot_model.inertias[joint_id].lever();
    }

    Eigen::Matrix3d Aliengo::getLinkInertia(const std::string& name) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(name)].parentJoint;
        return robot_model.inertias[joint_id].inertia().matrix();
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