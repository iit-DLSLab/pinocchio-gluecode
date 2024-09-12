#include "controllers/pinocchio_controller/pinocchio_controller.hpp"
#include "controllers/pinocchio_controller/topics.hpp"
// topics
#include <dls2/topics/topics.hpp> // off-the-shelf topics
#include "dls2/math/rotations.hpp"
#include <robotlib/robot_factory.hpp>

// PINOCCHIO
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <pinocchio/algorithm/rnea.hpp>
namespace controllers
{
    PinocchioController::PinocchioController (const std::string& ID, const std::shared_ptr<robotlib::RobotBase> robot)
    : dls::PeriodicAppPlugin(ID)
    , pRobot(robot)
    , input_base_state(robot) // instantiate input
    , input_blind_state(robot) // instantiate input
    , output_tau(robot) // instantiate output
    , output_traj_gen(robot) // instantiate output
    , pose_increment(Eigen::Vector<double,6>::Zero())
    // , outFile ("")
    {
        // load pinocchio model from urdf
        const std::string urdf_name = "/usr/include/aliengo_description/urdfs/aliengo.urdf";
        pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), robot_model);
        robot_data = pinocchio::Data(robot_model);

        q_increment.resize(pRobot->getNJOINTS());

        // Create dynamic debug message
        // debug_msg = createDynamicMessage(dls::topics::pinocchio_controller::debug.second);
        // debug_msg->set_string_value("pinocchio_debug", debug_msg->get_member_id_by_name("frame_id"));

        this->buildInput<dls::BaseState>(
            dls::topics::high_level_estimation::base_state,
            &input_base_state
        );
        this->buildInput<dls::BlindState>(
            dls::topics::low_level_estimation::blind_state,
            &input_blind_state
        );
        // Define outputs
        this->buildOutput<dls::ControlSignal>(
            dls::topics::pinocchio_controller::tau,
            &output_tau
        );

        dds_participant_->addWriter("pinocchio_writer", dls::topics::pinocchio_controller::debug);

        // debug_msg.feet_position_robcogen() = std::vector<double>(pRobot->getNJOINTS(), 0.0);
        // debug_msg.feet_velocity_robcogen() = std::vector<double>(pRobot->getNJOINTS(), 0.0);
        // debug_msg.feet_position_pin() = std::vector<double>(pRobot->getNJOINTS(), 0.0);
        // debug_msg.feet_velocity_pin() = std::vector<double>(pRobot->getNJOINTS(), 0.0);

        command_manager.addCommand(  "set_q_increment",
                                            "set_q_increment",
                                            &PinocchioController::setQIncrement, this, {}, true);
        command_manager.addCommand(  "set_pose_increment",
                                            "set_pose_increment",
                                            &PinocchioController::setPoseIncrement, this, {}, true);
    }

    PinocchioController::~PinocchioController()
    { }

    std::string PinocchioController::where()
    {
		std::stringstream ss;
    ss << "PINOCCHIO CONTROLLER\n";

		return ss.str();
    }

    void PinocchioController::run(const std::chrono::system_clock::time_point& time)
    {
        // Read inputs
        read();

        // Run module
        runController();

        // compure desired joint configuration
        Eigen::Vector<double,12> q_home {0.0, 0.75, -1.5, 0.0, 0.75, -1.5, 0.0, 0.75, -1.5, 0.0, 0.75, -1.5};
        Eigen::VectorXd q_gt = pinocchio::neutral(robot_model);
        q_gt.tail(pRobot->getNJOINTS()) = reorderJoints(q_home) + q_increment;
        pinocchio::framesForwardKinematics(robot_model, robot_data, q_gt);

        // get desired end effector pose
		const std::string ee_name ("lf_foot");
        auto ee_id = robot_model.getFrameId(ee_name);
        auto oMee = robot_data.oMf[ee_id];
        oMee.translation()(0) += pose_increment(0);
        oMee.translation()(1) += pose_increment(1);
        oMee.translation()(2) += pose_increment(2);
        // orientation (RPY)
        auto oRee = oMee.rotation();
        for(int i=2; i>=0;i--){ // z,y,x rotations
            Eigen::Vector3d unit_vector = Eigen::Vector3d::Zero();
            unit_vector(i) = 1;
            Eigen::Vector3d r = oRee.transpose() * unit_vector;
            const double angle = pose_increment(i+3);
            Eigen::Matrix3d r_skew;
            r_skew << 0, -r(2), r(1),
                r(2), 0, -r(0),
                -r(1), r(0), 0;
            oRee *= r*r.transpose() + (Eigen::Matrix3d::Identity() - r*r.transpose())*cos(angle) + r_skew*sin(angle);
        }
        oMee.rotation() = oRee;
        // std::cout << dls::math::rotTorpy(oMee.rotation().transpose()).transpose() << std::endl;
        // get desired pose of end effector parent joint
        auto ee_joint_id = robot_model.frames[ee_id].parentJoint;
        auto oMiee = robot_data.oMi[ee_joint_id];
        auto oMiee_derived = oMee * robot_model.frames[ee_id].placement.inverse();// equal to robot_data.oMi[ee_joint_id]

        // auto oMides = oMiee_derived;
        auto oMides = oMee;

        debug_msg.ee_pose_des()[0] = oMee.translation()(0);
        debug_msg.ee_pose_des()[1] = oMee.translation()(1);
        debug_msg.ee_pose_des()[2] = oMee.translation()(2);
        auto rpy_ee = dls::math::rotTorpy(oMee.rotation().transpose());
        debug_msg.ee_pose_des()[3] = rpy_ee(0);
        debug_msg.ee_pose_des()[4] = rpy_ee(1);
        debug_msg.ee_pose_des()[5] = rpy_ee(2);

		Eigen::Matrix4d pose_des = Eigen::Matrix4d::Zero();
		pose_des.block<3,1>(0,3) = oMides.translation();
        pose_des.block<3,3>(0,0) = oMides.rotation();
        // ik
        Eigen::VectorXd q_pin = pinocchio::neutral(robot_model);
        q_pin.tail(pRobot->getNJOINTS()) = reorderJoints(input_blind_state.joints_position_.vec_());
        auto q_init_guess = q_gt;
        auto ee_parent_joint_name = robot_model.names[ee_joint_id];
		auto q_des = q_init_guess;
        q_des = inverseKinematicsFrame(ee_name, q_init_guess, q_gt, pose_des);
        
        // ik using robotlib
        robotlib::JointState q_des_robotlib = pRobot->makeJointState(0.0);
        robotlib::JointState q_init_guess_robotlib =  pRobot->makeJointState(0.0);
        q_init_guess_robotlib = reorderJoints(q_init_guess.tail(pRobot->getNJOINTS()));
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        pRobot->fixedBaseInveseKinematics(ee_name, q_init_guess_robotlib, pose_des.block<3,1>(0,3), q_des_robotlib);
        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        // outFile << std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0) <<std::endl;

        // std::map<std::string, Eigen::Vector3d> position_des_map = {
        //     {"lf_foot", robot_data.oMf[robot_model.getFrameId("lf_foot")].translation()+ pose_increment.head(3)},
        //     {"rf_foot", robot_data.oMf[robot_model.getFrameId("rf_foot")].translation()+ pose_increment.head(3)},
        //     {"lh_foot", robot_data.oMf[robot_model.getFrameId("lh_foot")].translation()+ pose_increment.head(3)},
        //     {"rh_foot", robot_data.oMf[robot_model.getFrameId("rh_foot")].translation()+ pose_increment.head(3)}
        // };
        // q_des_robotlib = q_init_guess_robotlib;
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        // for (auto &[name, position_des] : position_des_map){
        //     pRobot->fixedBaseInveseKinematics(name, q_des_robotlib, position_des, q_des_robotlib);
        // }
        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        // outFile << std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0) <<std::endl;

        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_inv_kin_gt().data(), debug_msg.q_inv_kin_gt().size()) = q_gt;
		Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_inv_kin().data(), debug_msg.q_inv_kin().size()) = q_des;
        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_inv_kin_robotlib().data(), debug_msg.q_inv_kin_robotlib().size()) = q_des_robotlib.vec_();

        // using robotlib solution as desired joint configuration
        q_des.tail(pRobot->getNJOINTS()) = reorderJoints(q_des_robotlib.vec_().tail(pRobot->getNJOINTS()));
        //compute control action
        Eigen::VectorXd err = q_des.tail(pRobot->getNJOINTS()) - q_pin.tail(pRobot->getNJOINTS());
        err = reorderJoints(err);
        Eigen::VectorXd qd = input_blind_state.joints_velocity_.vec_();
        output_tau.torques_ = 8*err + 1*(-1)*qd;

        dds_participant_->sendMessage("pinocchio_writer", &debug_msg);

        auto end_effector_position = pRobot->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        auto end_effector_velocity = pRobot->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        pRobot->forwardKinematics(  input_blind_state.joints_position_,
                                    input_blind_state.joints_velocity_,
                                    end_effector_position,
                                    end_effector_velocity
        );
        auto qd_des_robotlib = pRobot->makeJointState(0.0);
        pRobot->fixedBaseInverseDiffKinematics("lf_foot", input_blind_state.joints_position_, end_effector_velocity["LF"], qd_des_robotlib);
        // comment the function write() to test this with another joint controller
        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.qd_inv_diff_kin_robotlib().data(), debug_msg.qd_inv_diff_kin_robotlib().size()) = qd_des_robotlib.vec_();
        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.qd_gt().data(), debug_msg.qd_gt().size()) = input_blind_state.joints_velocity_.vec_();

        write();
    }

    bool PinocchioController::setQIncrement(){
        int idx = 0;

        dls::CommandHelper::readValue<int>("index", idx, idx);
        double increment = 0.0;
        if(dls::CommandHelper::readValue<double>("increment", increment, q_increment(idx))){
            q_increment(idx) = increment;
        }
        return true;
    }

    bool PinocchioController::setPoseIncrement(){
        int idx = 0;

        dls::CommandHelper::readValue<int>("index", idx, idx);
        double increment = 0.0;
        if(dls::CommandHelper::readValue<double>("increment", increment, pose_increment(idx))){
            pose_increment(idx) = increment;
        }
        return true;
    }
    bool PinocchioController::deactivation(const std::chrono::system_clock::time_point& time){
        output_tau.torques_.setZero();
        write();

        // outFile.close();
        return true;
    }

    void PinocchioController::runController(){}

    Eigen::VectorXd PinocchioController::reorderJoints(const Eigen::VectorXd& data) const{
        Eigen::VectorXd new_data = data;
        for(auto &[key, value] : idx_map)
        {
            new_data[value] = data[key];
            new_data[key] = data[value];
        }
        return new_data;
    }

    Eigen::VectorXd PinocchioController::inverseKinematics(const std::string& ee_parent_joint_name, const Eigen::VectorXd& q_guess, const Eigen::VectorXd& q_gt, const Eigen::Matrix4d& pose_des)
	{
		const int JOINT_ID = robot_model.getJointId(ee_parent_joint_name);
		const pinocchio::SE3 oMdes(pose_des.block<3,3>(0,0), pose_des.block<3,1>(0,3));

		const double eps = 1e-4;
		const int IT_MAX = 1000;
		const double DT = 1e-1;
		const double damp = 1e-6;
        // const double alpha = 0.001;
		pinocchio::Data::Matrix6x J(6, robot_model.nv);
		J.setZero();

		bool success = false;

        Eigen::Matrix<double, 6, 1> err = Eigen::Matrix<double, 6, 1>::Zero();
        auto q_des = q_guess;
		for (int i = 0;; i++)
		{
            // Eigen::VectorXd v(robot_model.nv);
            // v.setZero();
			// pinocchio::forwardKinematics(robot_model, robot_data, q_des);
			// const pinocchio::SE3 iMd = robot_data.oMi[JOINT_ID].actInv(oMdes);
			// err = pinocchio::log6(iMd).toVector(); // in joint frame
			// if (err.norm() < eps)
			// {
			// success = true;
			// break;
			// }
			// if (i >= IT_MAX)
			// {
			// success = false;
			// break;
			// }
			// pinocchio::computeJointJacobian(robot_model, robot_data, q_des, JOINT_ID, J); // J in joint frame
			// pinocchio::Data::Matrix6 Jlog;
			// pinocchio::Jlog6(iMd.inverse(), Jlog);
			// J = -Jlog * J;
            // pinocchio::Data::Matrix6 JJt;
			// JJt.noalias() = J * J.transpose();
			// JJt.diagonal().array() += damp;
			// v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
			// q_des = pinocchio::integrate(robot_model, q_des, v * DT);
			// if (!(i % 10))
			// std::cout << i << ": error = " << err.transpose() << std::endl;

            // Newton method: q_k+1 = q_k + J_pseudo*err, where err = f_task(q_des)- f_task(q_k) on position only
            pinocchio::forwardKinematics(robot_model, robot_data, q_des);
            err.head(3) = (robot_data.oMi[JOINT_ID].inverse() * oMdes).translation();
            if (err.head(3).norm() < eps)
			{
			success = true;
			break;
			}
			if (i >= IT_MAX)
			{
			success = false;
			break;
			}
			pinocchio::computeJointJacobian(robot_model, robot_data, q_des, JOINT_ID, J); // J in joint frame
            auto J_task = J.block<3,12>(0,0);

            Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
            JJt_task = J_task*J_task.transpose() + Eigen::MatrixXd::Identity(JJt_task.rows(),JJt_task.cols())*damp;
            auto J_task_pseudo = J_task.transpose()*(JJt_task.inverse());
            q_des = q_des + J_task_pseudo *err.head(3);

			// // pinocchio::Data::Matrix6 Jlog;
			// // pinocchio::Jlog6(iMd.inverse(), Jlog);
			// // J = -Jlog * J;
			// Eigen::Matrix3d JJt = Eigen::Matrix3d::Zero();
			// JJt.noalias() = J.block<3,12>(0,0) * J.block<3,12>(0,0).transpose();
			// JJt.diagonal().array() += damp;
            // // std::cout << J.block<3,12>(0,0).transpose()*JJt.inverse() << std::endl;
            // // std::cout << "--------------------------" << std::endl;
            // // std::cout << (J.block<3,12>(0,0).transpose()*JJt.inverse()*err.head(3)).transpose() << std::endl;
            // // std::cout << "................................" << std::endl;
			// // v.noalias() = -J.block<3,12>(0,0).transpose() * JJt.ldlt().solve(err.head(3));
            // // v.noalias() = J.block<3,12>(0,0).transpose()*JJt.inverse()*err.head(3);
			// // q_des = pinocchio::integrate(robot_model, q_des, v * DT);
            // // q_des += J.block<3,12>(0,0).transpose()*JJt.inverse()*err.head(3);
            // q_des += alpha*J.block<3,12>(0,0).transpose()*err.head(3);
            // // std::cout << q_des.transpose() << std::endl;
            // // std::cout << ".........." << std::endl;
            // // std::cout << err.transpose() << std::endl;
            // // std::cout << "----------------" << std::endl;
		}

		if (success)
		{
			// std::cout << "Convergence achieved!" << std::endl;
		}
		else
		{
			std::cout
			<< "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
			<< std::endl;
		}

		// std::cout << "\nresult: " << q_guess.transpose() << std::endl;
		// std::cout << "\ngroung truth: " << q_gt.transpose() << std::endl;
		// std::cout << "\nfinal error: " << err.transpose() << std::endl;
        Eigen::Matrix<double,6,1> err_base = Eigen::Matrix<double,6,1>::Zero();

        err_base.head(3) = robot_data.oMi[JOINT_ID].rotation()*err.head(3);

        for(int i=0; i<err.size();i++){
            debug_msg.err_inv_kin()[i] = err_base(i);
        }
        return q_des;
	}

    Eigen::VectorXd PinocchioController::inverseKinematicsFrame(const std::string& ee_name, const Eigen::VectorXd& q_guess, const Eigen::VectorXd& q_gt, const Eigen::Matrix4d& pose_des)
	{
		const int frame_id = robot_model.getFrameId(ee_name);
        pinocchio::FrameIndex base_frame_id = robot_model.getFrameId("base_link"); //base frame
		const pinocchio::SE3 oMdes(pose_des.block<3,3>(0,0), pose_des.block<3,1>(0,3));

		const double eps = 1e-4;
		const int IT_MAX = 1000;
		const double DT = 1e-2;
		const double damp = 1e-6;
		pinocchio::Data::Matrix6x J(6, robot_model.nv);
		J.setZero();

		bool success = false;

        Eigen::Vector3d err_task = Eigen::Vector3d::Zero();
        Eigen::VectorXd err_task_aug = Eigen::VectorXd::Zero(6);
        auto q_des = q_guess;
		for (int i = 0;; i++)
		{

            pinocchio::framesForwardKinematics(robot_model, robot_data, q_des);
			const pinocchio::SE3 fMd = robot_data.oMf[frame_id].actInv(oMdes);
            // auto err_pose = pinocchio::log6(fMd).toVector(); // in ee frame;


            Eigen::Matrix3d b_R_o = robot_data.oMf[base_frame_id].inverse().rotation();
            err_task = b_R_o*(oMdes.translation()) - b_R_o*(robot_data.oMf[frame_id].translation()-robot_data.oMf[base_frame_id].translation()); 
            // if (err.head(3).norm() < eps)
            if (err_task.norm() < eps)
			{
                success = true;
                debug_msg.ee_pose()[0] = robot_data.oMf[frame_id].translation()(0);
                debug_msg.ee_pose()[1] = robot_data.oMf[frame_id].translation()(1);
                debug_msg.ee_pose()[2] = robot_data.oMf[frame_id].translation()(2);
                auto rpy_ee_parent_joint = dls::math::rotTorpy(robot_data.oMf[frame_id].rotation().transpose());
                debug_msg.ee_pose()[3] = rpy_ee_parent_joint(0);
                debug_msg.ee_pose()[4] = rpy_ee_parent_joint(1);
                debug_msg.ee_pose()[5] = rpy_ee_parent_joint(2);
                break;
			}
			if (i >= IT_MAX)
			{
                success = false;
                break;
			}
			pinocchio::computeFrameJacobian(robot_model, robot_data, q_des, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
            Eigen::MatrixXd J_task = Eigen::MatrixXd::Zero(3,robot_model.nv);
            J_task.block<3,18>(0,0) = J.block<3,18>(0,0); //x, y, z
            J_task.block<3,6>(0,0).setZero(); //the derivative of the task is the derivative of the error w.r.t. base frame
            // Newton method: q_k+1 = q_k + J_pseudo*err_task, where err_task = f_task(q_des)- f_task(q_k) on position only
            Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
            JJt_task = J_task*J_task.transpose() + Eigen::MatrixXd::Identity(JJt_task.rows(),JJt_task.cols())*damp;
            auto J_task_pseudo = J_task.transpose()*(JJt_task.inverse());
            // q_des.tail(pRobot->getNJOINTS()) = q_des.tail(pRobot->getNJOINTS()) + (J_task_pseudo *err_task).tail(pRobot->getNJOINTS()); 
            // Eigen::VectorXd v = robot_data.oMf(J_task_pseudo * err_task);
			// q_des = pinocchio::integrate(robot_model, q_des, J_task_pseudo * err_task);
            Eigen::VectorXd qd_des = J_task_pseudo * err_task;
            // CLIK + add manage redundancy, possibly with projected/reduced gradient to be far from joint limits
            q_des.tail(pRobot->getNJOINTS()) = q_des.tail(pRobot->getNJOINTS()) + (qd_des).tail(pRobot->getNJOINTS())*DT; //CLIK

            // pinocchio::framesForwardKinematics(robot_model, robot_data, q_des);
			// const pinocchio::SE3 fMd = robot_data.oMf[frame_id].actInv(oMdes);
            // auto err_pose = pinocchio::log6(fMd).toVector(); // in ee frame;
            // // auto err_task = err_pose.tail(3);

            // err_task_aug.head(3) = robot_data.oMf[frame_id].rotation() * err_pose.head(3);
            // std::cout << q_des.head(3).transpose() << std::endl;
            // err_task_aug.tail(3) = Eigen::Vector3d::Zero() - q_des.head(3);
            // std::cout << "------------" << std::endl;
            // std::cout << err_task_aug.transpose() << std::endl;

            // // if (err.head(3).norm() < eps)
            // if (err_task_aug.norm() < eps)
			// {
			// success = true;
            // // std::cout << err_pose.transpose() << std::endl;
            // debug_msg.ee_parent_joint_pose_des()[0] = robot_data.oMf[frame_id].translation()(0);
            // debug_msg.ee_parent_joint_pose_des()[1] = robot_data.oMf[frame_id].translation()(1);
            // debug_msg.ee_parent_joint_pose_des()[2] = robot_data.oMf[frame_id].translation()(2);
            // auto rpy_ee_parent_joint = dls::math::rotTorpy(robot_data.oMf[frame_id].rotation().transpose());
            // debug_msg.ee_parent_joint_pose_des()[3] = rpy_ee_parent_joint(0);
            // debug_msg.ee_parent_joint_pose_des()[4] = rpy_ee_parent_joint(1);
            // debug_msg.ee_parent_joint_pose_des()[5] = rpy_ee_parent_joint(2);
			// break;
			// }
			// if (i >= IT_MAX)
			// {
			// success = false;
			// break;
			// }

			// pinocchio::computeJointJacobians(robot_model, robot_data, q_des);
            // pinocchio::getFrameJacobian(robot_model,robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J); // foot position task
            // Eigen::MatrixXd J_task1 =  J.block<3,18>(0,0);//x, y, z

            // pinocchio::getFrameJacobian(robot_model,robot_data, base_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
            // Eigen::MatrixXd J_task2 =  J.block<3,18>(0,0);//x, y, z
            
            // Eigen::MatrixXd J_task = Eigen::MatrixXd::Zero(6,robot_model.nv);
            // J_task.block<3,18>(0,0) = J_task1; // foot position
            // J_task.block<3,18>(3,0) = J_task2; // base position

            // // // Newton method: q_k+1 = q_k + J_pseudo*err_task, where err_task = f_task(q_des)- f_task(q_k) on position only
            // Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
            // JJt_task = J_task*J_task.transpose() + Eigen::MatrixXd::Identity(JJt_task.rows(),JJt_task.cols())*damp;
            // auto J_task_pseudo = J_task.transpose()*(JJt_task.inverse());
            // // q_des.tail(pRobot->getNJOINTS()) = q_des.tail(pRobot->getNJOINTS()) + (J_task_pseudo *err_task).tail(pRobot->getNJOINTS());

			// q_des = pinocchio::integrate(robot_model, q_des, J_task_pseudo * err_task_aug);
		}
        
        // std::cout << "**********************" << std::endl;

		if (success)
		{
			// std::cout << "Convergence achieved!" << std::endl;
		}
		else
		{
			std::cout
			<< "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
			<< std::endl;
		}

        debug_msg.err_inv_kin()[0] = err_task(0);
        debug_msg.err_inv_kin()[1] = err_task(1);
        debug_msg.err_inv_kin()[2] = err_task(2);
        debug_msg.err_inv_kin()[3] = err_task(3);
        debug_msg.err_inv_kin()[4] = err_task(4);
        debug_msg.err_inv_kin()[5] = err_task(5);

        return q_des;
	}

    extern "C" PeriodicAppPlugin *create(const std::string& ID, const std::string& robot_name)
    {
        if (robot_name == "")
        {
            std::string e = "Parameter robot_name is not defined, verify if the parameter server is running";
            throw std::runtime_error(e);
        }

        std::shared_ptr<robotlib::RobotBase> pRobot;
        try
        {
            pRobot = robotlib::RobotFactory::openRobot(robot_name);
        }
        catch (const std::exception &e)
        {
            std::cerr << "child_process: Could not open the robot " << robot_name << std::endl;
            std::cerr << e.what() << std::endl;
        }

        return new PinocchioController(ID, pRobot);
    }

    extern "C" void destroy(PeriodicAppPlugin *p)
    {
            delete p;
    }
} //namespace controllers
