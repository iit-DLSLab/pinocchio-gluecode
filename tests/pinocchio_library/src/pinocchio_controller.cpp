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
    {
        // load pinocchio model from urdf
        const std::string urdf_name = "/usr/include/aliengo_description/urdfs/aliengo.urdf";
        pinocchio::urdf::buildModel(urdf_name, robot_model);
        robot_data = pinocchio::Data(robot_model);

        q_increment.resize(robot_model.nq);

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

        // this->buildOutput<dls::TrajectoryGenerator>(
        //     dls::topics::trajectory_generator,
        //     &output_traj_gen
        // );
        // create raw pinocchio writer
        // dds_participant_->addWriter("pinocchio_writer", dls::topics::pinocchio_controller::debug);
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

        Eigen::VectorXd q_pin = input_blind_state.joints_position_.vec_();
        reoderJoints(q_pin);

        // compure desired joint configuration
        Eigen::Vector<double,12> q_home {0.0, 0.75, -1.5, 0.0, 0.75, -1.5, 0.0, 0.75, -1.5, 0.0, 0.75, -1.5};
        auto q_gt = q_home + q_increment;
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

        debug_msg.ee_pose_des()[0] = oMiee.translation()(0);
        debug_msg.ee_pose_des()[1] = oMiee.translation()(1);
        debug_msg.ee_pose_des()[2] = oMiee.translation()(2);
        auto rpy_ee = dls::math::rotTorpy(oMiee.rotation().transpose());
        debug_msg.ee_pose_des()[3] = rpy_ee(0);
        debug_msg.ee_pose_des()[4] = rpy_ee(1);
        debug_msg.ee_pose_des()[5] = rpy_ee(2);

        debug_msg.ee_parent_joint_pose_des()[0] = oMiee_derived.translation()(0);
        debug_msg.ee_parent_joint_pose_des()[1] = oMiee_derived.translation()(1);
        debug_msg.ee_parent_joint_pose_des()[2] = oMiee_derived.translation()(2);
        auto rpy_ee_parent_joint = dls::math::rotTorpy(oMiee_derived.rotation().transpose());
        debug_msg.ee_parent_joint_pose_des()[3] = rpy_ee_parent_joint(0);
        debug_msg.ee_parent_joint_pose_des()[4] = rpy_ee_parent_joint(1);
        debug_msg.ee_parent_joint_pose_des()[5] = rpy_ee_parent_joint(2);

		Eigen::Matrix4d pose_des = Eigen::Matrix4d::Zero();
		pose_des.block<3,1>(0,3) = oMides.translation();
        pose_des.block<3,3>(0,0) = oMides.rotation();
        // ik
        auto q_init_guess = q_pin;
        auto ee_parent_joint_name = robot_model.names[ee_joint_id];
		// auto q_des = inverseKinematics(ee_parent_joint_name, q_init_guess, q_gt, pose_des);
		auto q_des = inverseKinematicsFrame(ee_name, q_init_guess, q_gt, pose_des);

        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_inv_kin_gt().data(), debug_msg.q_inv_kin_gt().size()) = q_gt;
		Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_inv_kin().data(), debug_msg.q_inv_kin().size()) = q_des;
        Eigen::Vector<double, Eigen::Dynamic>::Map(debug_msg.q_init_guess().data(), debug_msg.q_init_guess().size()) = q_init_guess;

        //compute control action
        Eigen::VectorXd err = q_des - q_pin;
        reoderJoints(err);
        Eigen::VectorXd qd = input_blind_state.joints_velocity_.vec_();
        output_tau.torques_ = 15*err + 3*(-1)*qd;


        dds_participant_->sendMessage("pinocchio_writer", &debug_msg);


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
        return true;
    }

    void PinocchioController::runController(){}

    void PinocchioController::reoderJoints(Eigen::VectorXd& data) const{
        auto old_data = data;
        for(auto &[key, value] : idx_map)
        {
            data[value] = old_data[key];
            data[key] = old_data[value];
        }
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
			// pinocchio::framesForwardKinematics(robot_model, robot_data, q_des);
			// const pinocchio::SE3 fMd = robot_data.oMf[frame_id].actInv(oMdes);
			// err = pinocchio::log6(fMd).toVector(); // in base frame
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
			// pinocchio::computeFrameJacobian(robot_model, robot_data, q_des, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
			// pinocchio::Data::Matrix6 Jlog;
			// pinocchio::Jlog6(fMd.inverse(), Jlog);
			// J = -Jlog * J;
            // pinocchio::Data::Matrix6 JJt;
			// JJt.noalias() = J * J.transpose();
			// JJt.diagonal().array() += damp;
			// v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
			// q_des = pinocchio::integrate(robot_model, q_des, v * DT);

            pinocchio::framesForwardKinematics(robot_model, robot_data, q_des);
			const pinocchio::SE3 fMd = robot_data.oMf[frame_id].actInv(oMdes);
            auto err_pose = pinocchio::log6(fMd).toVector(); // in ee frame;
            // auto err_task = err_pose.tail(3);
            Eigen::Vector3d err_task(err_pose(0), err_pose(2), err_pose(3));
            // if (err.head(3).norm() < eps)
            if (err_task.norm() < eps)
			{
			success = true;
			break;
			}
			if (i >= IT_MAX)
			{
			success = false;
			break;
			}
			pinocchio::computeFrameJacobian(robot_model, robot_data, q_des, frame_id, pinocchio::LOCAL, J);
            // auto J_task = J.block<3,12>(0,0);
            // auto J_task = J.block<3,12>(3,0);
            Eigen::Matrix<double,3,12> J_task = Eigen::Matrix<double,3,12>::Zero();
            J_task.block<1,12>(0,0) = J.block<1,12>(0,0); //x
            J_task.block<1,12>(1,0) = J.block<1,12>(2,0); //z
            J_task.block<1,12>(2,0) = J.block<1,12>(3,0); // roll

            // Newton method: q_k+1 = q_k + J_pseudo*err_task, where err_task = f_task(q_des)- f_task(q_k) on position only
            Eigen::MatrixXd JJt_task = Eigen::MatrixXd::Zero(J_task.rows(), J_task.rows());
            JJt_task = J_task*J_task.transpose() + Eigen::MatrixXd::Identity(JJt_task.rows(),JJt_task.cols())*damp;
            auto J_task_pseudo = J_task.transpose()*(JJt_task.inverse());
            q_des = q_des + J_task_pseudo *err_task;
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

        Eigen::Matrix<double,6,1> err_base = Eigen::Matrix<double,6,1>::Zero();

        err_base.head(3) = err.head(3);

        for(int i=0; i<err.size();i++){
            debug_msg.err_inv_kin()[i] = err_base(i);
        }
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
