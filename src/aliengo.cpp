#include "aliengolib/aliengo.hpp"

#include "robotlib/utils/utils.hpp"

// #include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"
// #include <algorithm>
// #include <cctype>
// #include <cmath>
// #include <iterator>
// #include <memory>
// #include <numeric>
#include <pinocchio/algorithm/rnea.hpp>

#include "robotlib/utils/eigen_utils.hpp"

#include <filesystem>
#include <fstream>
#include <vector>

namespace aliengolib
{
    Aliengo::Aliengo(const YAML::Node& kinematics_mapping) : Robot()
    {
        auto limbs_definition = loadLimbsDefinition(kinematics_mapping);        
        // get robot name
        std::string robot_name = kinematics_mapping["name"].as<std::string>();
        // get trunk name
        // std::string trunk_name = kinematics_mapping["trunk"].as<std::string>();
        const robotlib::DynParams trunk_dyn_params{Eigen::Vector3d::Zero(), 5, Eigen::Matrix3d::Zero()}; //dummy com, mass, inertia
        // get limbs
        std::vector<robotlib::LimbPtr> limbs;
        for (auto const& limb : limbs_definition)
        {
            limbs.push_back(std::make_shared<robotlib::Limb>(limb.at("name")[0], limb.at("dls_links_name"), limb.at("dls_joints_name"), limb.at("type")[0]));
        }

        // initialize robot
        init(robot_name, trunk_dyn_params, limbs);

        // dls name to urdf ones
        for (const auto& limb : limbs_definition) {
            for (int i=0; i<limb.at("dls_joints_name").size();i++) {
                std::string dls_joint_name = limb.at("dls_joints_name")[i];
                std::string urdf_joint_name = limb.at("urdf_joints_name")[i];
                dls_to_urdf_joints_name[dls_joint_name] = urdf_joint_name;
            }
            for (int i=0; i<limb.at("dls_links_name").size();i++) {
                std::string dls_link_name = limb.at("dls_links_name")[i];
                std::string urdf_link_name = limb.at("urdf_links_name")[i];
                dls_to_urdf_links_name[dls_link_name] = urdf_link_name;
            }
        }
        // read joint direction from yaml
        for (const auto& limb : limbs_definition) {
            for (const auto& direction_str : limb.at("joints_direction")) {
                joint_directions.push_back(std::stoi(direction_str));
            }
        }

        // load pinocchio model from urdf
        const std::string urdf_name = kinematics_mapping["urdf_path"].as<std::string>();
        pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), robot_model);
        robot_data = pinocchio::Data(robot_model);

        // from order of joints defined in the yaml, extract the mapping between this order and the pinocchio one
        // dls_joint_order (urdf names)
        std::vector<std::string> dls_joint_order;
        for (const auto& limb : limbs_definition) {
            for (const auto& joint_name : limb.at("urdf_joints_name")) {
                dls_joint_order.push_back(joint_name);
            }
        }
        // find the mapping between the dls and pinocchio order using urdf names
        std::vector<std::string> pinocchio_joint_order;
        for (int i=0; i< dls_joint_order.size();i++) {
            int pinocchio_idx = getJointIdWithoutRoot(dls_joint_order[i]);
            idx_map[i] = pinocchio_idx;
        }

        setJointLimitsFromUrdf();
    }

    int Aliengo::getJointIdWithoutRoot(const std::string &joint_name) const
    {
        return robot_model.getJointId(joint_name)-2;//2: the number of joints are universe+root_joint+robot_joints
    }

    LimbList Aliengo::loadLimbsDefinition(const YAML::Node& root)
    {
        LimbList limbs_definition;

        if (!root["limbs"] || !root["limbs"].IsSequence()) {
            throw std::runtime_error("YAML must contain a 'limbs' sequence");
        }

        for (const auto& limb_node : root["limbs"]) {
            LimbMap limb_def;

            // limb short name, e.g. "LF", "RF", ...
            const std::string limb_name = limb_node["name"].as<std::string>();
            limb_def["name"].push_back(limb_name);

            // type: e.g. "leg"
            if (limb_node["type"]) {
                limb_def["type"].push_back(limb_node["type"].as<std::string>());
            }

            // --- joints: build "LF_HAA", "LF_HFE", "LF_KFE", ...
            const YAML::Node& joints = limb_node["chain"]["joints"];
            if (joints && joints.IsSequence()) {
                for (const auto& j : joints) {
                    // j["name"] is e.g. "HAA", "HFE", "KFE"
                    std::string j_name = j["name"].as<std::string>();
                    std::string j_name_urdf = j["urdf"].as<std::string>();
                    std::string j_direction = j["direction"].as<std::string>();
                    limb_def["dls_joints_name"].push_back(limb_name + "_" + j_name);
                    limb_def["urdf_joints_name"].push_back(j_name_urdf);
                    limb_def["joints_direction"].push_back(j_direction);
                }
            }

            // --- links: build "LF_ASSEMBLY", "LF_UPPERLERG", "LF_LOWERLEG", "LF_FOOT", ...
            const YAML::Node& links = limb_node["chain"]["links"];
            if (links && links.IsSequence()) {
                for (const auto& l : links) {
                    std::string l_name = l["name"].as<std::string>();
                    std::string l_name_urdf = l["urdf"].as<std::string>();
                    limb_def["dls_links_name"].push_back(limb_name + "_" + l_name);
                    limb_def["urdf_links_name"].push_back(l_name_urdf);
                }
            }

            limbs_definition.push_back(std::move(limb_def));
        }

        return limbs_definition;
    }

    Aliengo::~Aliengo(){}

    void Aliengo::setJointLimitsFromUrdf()
    {
        for(auto& joint : this->joints_){
                // **Transform robotlib joint name to urdf one**
                std::string urdf_joint_name = dls_to_urdf_joints_name[joint->getName()];

                const int joint_id_nq = getJointIdForNq(urdf_joint_name);
                const int joint_id_nv = getJointIdForNv(urdf_joint_name);
                const double q_min = robot_model.lowerPositionLimit[joint_id_nq];
                const double q_max = robot_model.upperPositionLimit[joint_id_nq];
                const double qd_max = robot_model.velocityLimit[joint_id_nv];
                const double tau_max = robot_model.effortLimit[joint_id_nv];
                joint->setJointLimits(q_min, q_max, qd_max, tau_max);
            }
    }

    int Aliengo::getJointIdForNq(const std::string &joint_name) const
    {
        return robot_model.getJointId(joint_name)+5;//5: in any variable of dimension nq, with a model having a JointModelFreeFlyer as root_joint, the first 7 values corresponds to the floating base joint (pos,quaternion). But the number of joints are universe+root_joint+robot_joints. So the index of the first robot joint is 2, which corresponds to 7 in any variable of dimension nq. So we use the joint_id offset=5.
    }

    int Aliengo::getJointIdForNv(const std::string &joint_name) const
    {
        return robot_model.getJointId(joint_name)+4;//4: in any variable of dimension nv, with a model having a JointModelFreeFlyer as root_joint, the first 6 values corresponds to the floating base joint (lin. vel.,ang. vel.). But the number of joints are universe+root_joint+robot_joints. So the index of the first robot joint is 2, which corresponds to 6 in any variable of dimension nq. So we use the joint_id offset=4.
    }

    Eigen::Vector3d Aliengo::computeFramePosition(  const robotlib::JointState &q,
                                                const robotlib::FramePtr origin,
                                                const robotlib::FramePtr destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();

        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);

        auto origin_frame_idx = robot_model.getFrameId(origin->getName());
        auto dest_frame_idx = robot_model.getFrameId(destination->getName());
        auto oMorigin = robot_data.oMf[origin_frame_idx];
        auto oMdest = robot_data.oMf[dest_frame_idx];

        return (oMdest.inverse() * oMorigin).translation();
    }

    Eigen::Matrix3d Aliengo::computeFrameOrientation( const robotlib::JointState &q,
                                                const robotlib::FramePtr origin,
                                                const robotlib::FramePtr destination)
    {
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_pin);
        // std::string origin_name = origin->getName();
        // std::string destination_name = destination->getName();

        // transform(origin_name.begin(), origin_name.end(), origin_name.begin(), ::tolower);
        // transform(destination_name.begin(), destination_name.end(), destination_name.begin(), ::tolower);

        auto origin_frame_idx = robot_model.getFrameId(origin->getName());
        auto dest_frame_idx = robot_model.getFrameId(destination->getName());
        auto oMorigin = robot_data.oMf[origin_frame_idx];
        auto oMdest = robot_data.oMf[dest_frame_idx];

        return (oMdest.inverse() * oMorigin).rotation();
    }

    Eigen::Matrix4d Aliengo::computeFramePose(const robotlib::JointState &q,
                                            const robotlib::FramePtr origin,
                                            const robotlib::FramePtr destination)
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << computeFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << computeFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }

    void Aliengo::computeLimbsJacobian(const robotlib::JointState &q,
                                        const robotlib::FramePtr frame,
                                        Eigen::MatrixXd &jacobian){
        computeLimbsJacobian(q, frame->getName(), jacobian);
    }

    void Aliengo::computeLimbsJacobian(const robotlib::JointState &q,
                                        const std::string& frame_name,
                                        Eigen::MatrixXd &jacobian){
        // map robotlib to pinocchio
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);

        // compute jacobian in base frame: since we are setting the robot pose to Identity (using fromRobotlibToPinocchioJointState(q)) the jacobian computed in LOCAL_WORLD_ALIGNED is the jacobian in the base frame
        
        const std::string urdf_frame_name = dls_to_urdf_links_name.at(frame_name);
        const int frame_id = robot_model.getFrameId(urdf_frame_name);
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
        pinocchio::computeFrameJacobian(robot_model, robot_data, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // set output
        const int jacobian_cols = robot_model.nv-6; // -6 because we are not considering the floating base joint
        jacobian = J.block(0,6,6, jacobian_cols); //6 because it is a geometric jacobian (lin, ang)

        jacobian = reorderLimbsJacobian(jacobian);
    }

    void Aliengo::computeWholeBodyJacobian(const Eigen::Matrix<double, 7, 1> &robot_pose,
                                            const robotlib::JointState &q,
                                            const robotlib::FramePtr frame,
                                            Eigen::MatrixXd &jacobian){
        // map robotlib to pinocchio
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(robot_pose, q);

        // compute jacobian in world frame
        const int frame_id = robot_model.getFrameId(frame->getName());
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
        pinocchio::computeFrameJacobian(robot_model, robot_data, q_pin, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // compute jacobian in base frame
        Eigen::Matrix3d b_R_w = robotlib::utils::quatToRotMat(Eigen::Quaterniond(q_pin.block<4,1>(3,0))); // orientation of the world frame expressed in base frame
        jacobian.resize(6, robot_model.nv);
        jacobian.block(0,0,3, robot_model.nv) = b_R_w * J.block(0,0,3, robot_model.nv);
        jacobian.block(3,0,3, robot_model.nv) = b_R_w * J.block(3,0,3, robot_model.nv);

        jacobian = reorderWholeBodyJacobian(jacobian);
    }


    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position);
        return q;
    }

    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointState(const Eigen::Matrix<double, 7, 1> &robot_pose, const robotlib::JointState &joint_position){
        Eigen::VectorXd q = pinocchio::neutral(robot_model);
        q.tail(this->getNJOINTS()) = reorderJoints(joint_position);
        q.head(7) = robot_pose;
        return q;
    }

    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity);
        return qd;
    }

    Eigen::VectorXd Aliengo::fromRobotlibToPinocchioJointVelocity(const Eigen::Matrix<double, 6, 1> &robot_velocity, const robotlib::JointState &joint_velocity){
        Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
        qd.tail(this->getNJOINTS()) = reorderJoints(joint_velocity);
        qd.head(6) = robot_velocity;
        return qd;
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    robotlib::LimbDataMap<Eigen::Vector3d> &end_effector_position)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);

        pinocchio::forwardKinematics(robot_model, robot_data, q);
        pinocchio::updateFramePlacements(robot_model, robot_data);

        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId(robot_model.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data.oMf[base_frame_id].inverse();

        // feet positions
        for(auto limb : limbs_){
            const std::string end_effector_name = dls_to_urdf_links_name.at(limb->getEndEffector()->getName());
            const int frame_id = robot_model.getFrameId(end_effector_name);
            end_effector_position[limb] = (baseMo*robot_data.oMf[frame_id]).translation();
        }
        // end_effector_position[this->getLimb("LF")] = (baseMo*robot_data.oMf[robot_model.getFrameId("lf_foot")]).translation();
        // end_effector_position[this->getLimb("RF")] = (baseMo*robot_data.oMf[robot_model.getFrameId("rf_foot")]).translation();
        // end_effector_position[this->getLimb("LH")] = (baseMo*robot_data.oMf[robot_model.getFrameId("lh_foot")]).translation();
        // end_effector_position[this->getLimb("RH")] = (baseMo*robot_data.oMf[robot_model.getFrameId("rh_foot")]).translation();
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    robotlib::LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LimbDataMap<Eigen::Vector3d> &end_effector_velocity)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(joint_velocity);

        pinocchio::forwardKinematics(robot_model, robot_data, q, qd);
        pinocchio::updateFramePlacements(robot_model, robot_data);

        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId(robot_model.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data.oMf[base_frame_id].inverse();

        // feet positions
        for(auto limb : limbs_){
            const std::string end_effector_name = dls_to_urdf_links_name.at(limb->getEndEffector()->getName());
            const int frame_id = robot_model.getFrameId(end_effector_name);
            end_effector_position[limb] = (baseMo*robot_data.oMf[frame_id]).translation();
            end_effector_velocity[limb] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        }
        // end_effector_position[this->getLimb("LF")] = (baseMo*robot_data.oMf[robot_model.getFrameId("lf_foot")]).translation();
        // end_effector_position[this->getLimb("RF")] = (baseMo*robot_data.oMf[robot_model.getFrameId("rf_foot")]).translation();
        // end_effector_position[this->getLimb("LH")] = (baseMo*robot_data.oMf[robot_model.getFrameId("lh_foot")]).translation();
        // end_effector_position[this->getLimb("RH")] = (baseMo*robot_data.oMf[robot_model.getFrameId("rh_foot")]).translation();

        // // feet velocities
        // end_effector_velocity[this->getLimb("LF")] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("lf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        // end_effector_velocity[this->getLimb("RF")] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("rf_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        // end_effector_velocity[this->getLimb("LH")] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("lh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
        // end_effector_velocity[this->getLimb("RH")] = baseMo.rotation()*pinocchio::getFrameVelocity(robot_model, robot_data, robot_model.getFrameId("rh_foot"), pinocchio::LOCAL_WORLD_ALIGNED).linear();
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    const robotlib::JointState &joint_acceleration,
                                    robotlib::LimbDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LimbDataMap<Eigen::Matrix3d> &end_effector_orientation,
                                    robotlib::LimbDataMap<robotlib::Vec6d> &end_effector_velocity,
                                    robotlib::LimbDataMap<robotlib::Vec6d> &end_effector_acceleration)
    {
        Eigen::VectorXd q = fromRobotlibToPinocchioJointState(joint_position);
        Eigen::VectorXd qd = fromRobotlibToPinocchioJointVelocity(joint_velocity);
        Eigen::VectorXd qdd = fromRobotlibToPinocchioJointVelocity(joint_acceleration);

        pinocchio::forwardKinematics(robot_model, robot_data, q, qd, qdd);
        pinocchio::updateFramePlacements(robot_model, robot_data);

        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId(robot_model.frames[2].name); //base frame
        pinocchio::SE3 baseMo = robot_data.oMf[base_frame_id].inverse();

        for(auto limb : limbs_){
            const std::string end_effector_name = dls_to_urdf_links_name.at(limb->getEndEffector()->getName());
            const int frame_id = robot_model.getFrameId(end_effector_name);
            end_effector_position[limb] = (baseMo*robot_data.oMf[frame_id]).translation();
            end_effector_orientation[limb] =
                (baseMo * robot_data.oMf[frame_id]).rotation();
            end_effector_velocity[limb].head(3) = baseMo.rotation()
                * pinocchio::getFrameVelocity(robot_model, robot_data, frame_id,
                                                pinocchio::LOCAL).angular();
            end_effector_velocity[limb].tail(3) = baseMo.rotation()
                * pinocchio::getFrameVelocity(robot_model, robot_data, frame_id,
                                                pinocchio::LOCAL).linear();
            end_effector_acceleration[limb].head(3) = baseMo.rotation()
                * pinocchio::getFrameAcceleration(robot_model, robot_data, frame_id,
                                                    pinocchio::LOCAL).angular();
            end_effector_acceleration[limb].tail(3) = baseMo.rotation()
                * pinocchio::getFrameAcceleration(robot_model, robot_data, frame_id,
                                                    pinocchio::LOCAL).linear();
        }

        // int frameId;
        // for (auto leg : {"LF", "RF", "LH", "RH"})
        // {
        //   frameId = robot_model.getFrameId(robotlib::utils::toLower(leg) + "_foot");
        //   end_effector_position[getLimb(leg)] =
        //       (baseMo * robot_data.oMf[frameId]).translation();
        //   end_effector_orientation[getLimb(leg)] =
        //       (baseMo * robot_data.oMf[frameId]).rotation();
        //   end_effector_velocity[getLimb(leg)].head(3) = baseMo.rotation()
        //       * pinocchio::getFrameVelocity(robot_model, robot_data, frameId,
        //                                     pinocchio::LOCAL).angular();
        //   end_effector_velocity[getLimb(leg)].tail(3) = baseMo.rotation()
        //       * pinocchio::getFrameVelocity(robot_model, robot_data, frameId,
        //                                     pinocchio::LOCAL).linear();
        //   end_effector_acceleration[getLimb(leg)].head(3) = baseMo.rotation()
        //       * pinocchio::getFrameAcceleration(robot_model, robot_data, frameId,
        //                                         pinocchio::LOCAL).angular();
        //   end_effector_acceleration[getLimb(leg)].tail(3) = baseMo.rotation()
        //       * pinocchio::getFrameAcceleration(robot_model, robot_data, frameId,
        //                                         pinocchio::LOCAL).linear();
        // }
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
        const std::string urdf_frame_name = dls_to_urdf_links_name.at(frame_name);
        // compute desired pose in pinocchio format
		const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), position_des);
        // map from robotlib to pinocchio
        Eigen::VectorXd q_pin_init_guess = fromRobotlibToPinocchioJointState(q_init_guess);

        // Closed Loop Inverse Kinematics (CLIK)
        Eigen::VectorXd q_pin_des = clik(urdf_frame_name, q_pin_init_guess, oMdes, IK::TASK::POSITION_TASK);

        // Get IK solution
        q_des = reorderJoints(q_pin_des.tail(this->getNJOINTS()));
    }

    void Aliengo::fixedBaseInverseKinematics(
        const robotlib::JointState &q_init_guess,
        const robotlib::LimbDataMap<Eigen::Vector3d> &positions_des,
        robotlib::JointState &q_des)
    {
        auto q_des_temp = this->makeJointState(0.0);
        for (auto &limb : limbs_)
        {
            std::string end_effector_name = limb->getEndEffector()->getName();
            this->fixedBaseInverseKinematics(end_effector_name, q_init_guess, positions_des.at(limb), q_des_temp);
            for (auto joint : limb->getJoints())
            {
                q_des[joint->id] = q_des_temp[joint->id];
            }
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
        const std::string urdf_frame_name = dls_to_urdf_links_name.at(frame_name);
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(urdf_frame_name);
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
                                        const robotlib::LimbDataMap<Eigen::Vector3d> &velocities_des,
                                        robotlib::JointState &qd_des)
    {
        auto qd_des_temp = this->makeJointState(0.0);
        for (auto &limb : limbs_)
        {
            std::string end_effector_name = limb->getEndEffector()->getName();
            this->fixedBaseInverseDiffKinematics(end_effector_name, q, velocities_des.at(limb), qd_des_temp);
            for (auto joint : limb->getJoints())
            {
                qd_des[joint->id] = qd_des_temp[joint->id];
            }
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
            const std::string urdf_frame_name = dls_to_urdf_links_name.at(frame_name);
            const pinocchio::FrameIndex frame_id = robot_model.getFrameId(urdf_frame_name);
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

    void Aliengo::computeJSInertiaMatrix( // TODO
        const Eigen::Matrix<double, 7, 1> &robot_pose,
        const robotlib::JointState &joint_position,
        Eigen::MatrixXd &jsInertia)
    {
      Eigen::VectorXd q = fromRobotlibToPinocchioJointState(robot_pose, joint_position);
      // pinocchio::computeMinverse(robot_model, robot_data, q);
      // jsInertia = robot_data.Minv;
      // // make triangular matrix symmetric
      // jsInertia.triangularView<Eigen::StrictlyLower>() =
      //     robot_data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
      pinocchio::crba(robot_model, robot_data, q);
      // reorder pinocchio to match robcogen: xyz, rpy, lf, rf, lh, rh, arm
      // make triangular matrix symmetric
      robot_data.M.triangularView<Eigen::StrictlyLower>() =
          robot_data.M.transpose().triangularView<Eigen::StrictlyLower>();
      // STI: only valid for urdf of hyqreal (4x3 dof) with arm (7 dof) and hand (6 dof)
      // ToDo: use a config file to get the mapping
      static std::vector<int> sizes = {3, 3, 3, 3, 3, 3}; //, 7, 6};
      static std::vector<int> indices = {3, 0, 6, 12, 9, 15}; //, 6, 13};
      static int all = std::accumulate(sizes.begin(), sizes.end(), 0);
      jsInertia.resize(all, all);
      jsInertia.setZero();

      // r: read, w: write, s: size, i: index, r: row, c: col --> rir = read index row
      int wir(0), wic, sr, sc, rir, ric;
      for (int i = 0; i < sizes.size(); i++)
      {
        wic = 0;
        sr = sizes[i];
        rir = indices[i];
        for (int j = 0; j < sizes.size(); j++)
        {
          sc = sizes[j];
          ric = indices[j];
          jsInertia.block(wir, wic, sr, sc) << robot_data.M.block(rir, ric, sr, sc);
          wic += sizes[j];
        }
        wir += sizes[i];
      }
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

    double Aliengo::getLinkMass(const robotlib::LinkPtr link) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(link->getName())].parentJoint;
        return robot_model.inertias[joint_id].mass();
    }

    double Aliengo::getRobotMass() const
    {
        return pinocchio::computeTotalMass(robot_model);
    }

    Eigen::Vector3d Aliengo::computeWholeBodyCoM(const robotlib::JointState& q) {
        // set the robot base pose to pos=0, ori=0 --> centerOfMass gives the CoM in the base frame
        Eigen::VectorXd q_pin = fromRobotlibToPinocchioJointState(q);
        return pinocchio::centerOfMass(robot_model, robot_data, q_pin, false);//false: do not compute com of subtrees
    }

    Eigen::Vector3d Aliengo::getLinkCoM(const robotlib::LinkPtr link) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(link->getName())].parentJoint;
        return robot_model.inertias[joint_id].lever();
    }

    Eigen::Matrix3d Aliengo::getLinkInertia(const robotlib::LinkPtr link) const
    {
        const int joint_id = robot_model.frames[robot_model.getFrameId(link->getName())].parentJoint;
        return robot_model.inertias[joint_id].inertia().matrix();
    }

    Eigen::VectorXd Aliengo::reorderJoints(const Eigen::VectorXd& data) const{
        Eigen::VectorXd new_data = data;
        for(auto &[key, value] : idx_map)
        {
            new_data(value) = data(key);
            new_data(key) = data(value);
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

    extern "C" std::shared_ptr<robotlib::RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf)
    {        
        return std::make_shared<Aliengo>(YAML::LoadFile("/usr/include/aliengo_description/kinematics/kinematics.yaml"));
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
