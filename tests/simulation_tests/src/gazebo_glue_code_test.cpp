#include "gazebo_glue_code_test.hpp"
#include <gazebo/sensors/sensors.hh>

#include <dls_messages/dds/gazebo_glue_code_testPubSubTypes.hpp>
#include <sstream>
#include <iostream>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/center-of-mass.hpp"

#include "robotlib/utils/eigen_utils.hpp"
namespace dls
{
	GazeboPluginGlueTest::GazeboPluginGlueTest() 
		: gazebo::ModelPlugin()
		, ddslink(std::make_shared<dls::DDSParticipant>(
			"GazeboPluginGlueTest::signals",
			dls::domains::signals
		))
	{
        const std::string urdf_name = "/usr/include/aliengo_description/urdfs/aliengo.urdf";
        pinocchio::urdf::buildModel(urdf_name, pinocchio::JointModelFreeFlyer(), robot_model);
        robot_data = pinocchio::Data(robot_model);

		ddslink->addWriter("gazebo_glue_code_test", dls::topicType("gazebo_glue_code_test", new GazeboGlueCodeTestPubSubType()));
	 }

	void GazeboPluginGlueTest::Load
	(
		gazebo::physics::ModelPtr model,
		sdf::ElementPtr element
	)
	{
		const std::string plugin_name {"GAZEBO_GLUECODE_TEST"};

		std::cout << "LOADING " << plugin_name << "PLUGIN" << std::endl;

		std::string robot_name {model->GetName()};

		if (!model)
		{
			std::cout << plugin_name << ": Parent model is NULL" << std::endl;
			return;
		}
		else{
			std::cout << plugin_name << ": MODEL " << robot_name << " IS INITIALIZED" << std::endl;
		}

		// Load robot model
		this->sim_model = model;
		std::string library_name = robot_name;
        try
        {
			if(library_name.find_last_of("_")!= std::string::npos)
			{
				library_name = library_name.substr(0, library_name.find_last_of("_"));
			}
            this->pRobot = robotlib::RobotFactory::openRobot(library_name);
        }
        catch (const std::exception &e)
        {
            std::cerr << plugin_name << ": Could not open the robot " << library_name << std::endl;
            std::cerr << e.what() << std::endl;
        }

		// Ordering joints in the same order defined in the robot specific library
		std::vector<gazebo::physics::JointPtr> sim_joints_urdf_order = model->GetJoints();
		for(auto leg: *this->pRobot->getLegs())
		{
			for(auto joint : *leg->getJoints())
			{
				// **Transform glue joint name to urdf one**
				std::string urdf_joint_name{joint->getName()};
				std::transform(urdf_joint_name.begin(), urdf_joint_name.end(), urdf_joint_name.begin(), ::tolower);
				urdf_joint_name.append("_joint");
				for(gazebo::physics::JointPtr sim_joint : sim_joints_urdf_order)
				{
					if(urdf_joint_name == sim_joint->GetName())
					{
						this->sim_joints.push_back(sim_joint);
						break;
					}
				}
			}
		}
		
		joints_positions = std::make_shared<robotlib::JointState>(pRobot->makeJointState(0.0));
		joints_velocity = std::make_shared<robotlib::JointState>(pRobot->makeJointState(0.0));
		joints_acceleration = std::make_shared<robotlib::JointState>(pRobot->makeJointState(0.0));
		joints_torques = std::make_shared<robotlib::JointState>(pRobot->makeJointState(0.0));

		this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin
		(
			std::bind(&GazeboPluginGlueTest::run, this)
		);

		std::cout << plugin_name << ": PLUGIN LOADED SUCCESSFULLY" << std::endl;
	}


    void GazeboPluginGlueTest::run()
	{	
		// get joints data
		int i=0;
		for(auto &leg : *joints_positions)
		{
			for(auto &joint : *leg.data_)
			{
				(*joints_positions)[joint.key_] = this->sim_joints[i]->Position();
				(*joints_velocity)[joint.key_] = this->sim_joints[i]->GetVelocity(0);
				(*joints_torques)[joint.key_] = this->sim_joints[i]->GetForce(0);
				i++;
			}
		}

		// get robot state
		base_pose = this->sim_model->GetLink(base_frame)->WorldPose();
		// -- robot position
		q = pinocchio::neutral(robot_model);
		q.block<3,1>(0,0) = Eigen::Vector3d(base_pose.Pos().X(), base_pose.Pos().Y(), base_pose.Pos().Z());
		Eigen::Quaterniond quaternion_base = dls::math::rpyToquat(Eigen::Vector3d(base_pose.Rot().Euler()[0], base_pose.Rot().Euler()[1], base_pose.Rot().Euler()[2]));
		q(3) = quaternion_base.x();
		q(4) = quaternion_base.y();
		q(5) = quaternion_base.z();
		q(6) = quaternion_base.w();
		q.tail(pRobot->getNJOINTS()) = reorderJoints(joints_positions->vec_());

		// -- robot velocity
		qd = Eigen::VectorXd::Zero(robot_model.nv);
		qd.tail(pRobot->getNJOINTS()) = reorderJoints(joints_velocity->vec_());
		auto w_base_lin_vel = this->sim_model->GetLink(base_frame)->WorldLinearVel();
		auto w_base_ang_vel = this->sim_model->GetLink(base_frame)->WorldAngularVel();
		auto b_base_lin_vel = base_pose.Rot().Inverse() * w_base_lin_vel;
		auto b_base_ang_vel = base_pose.Rot().Inverse() * w_base_ang_vel;
		qd.head(3) = Eigen::Vector3d(b_base_lin_vel.X(), b_base_lin_vel.Y(), b_base_lin_vel.Z());
		qd.segment(3,3) = Eigen::Vector3d(b_base_ang_vel.X(), b_base_ang_vel.Y(), b_base_ang_vel.Z());
		qdd = Eigen::VectorXd::Zero(robot_model.nv);

		// testForwardKinematics();

		// testGetPose();

		// testInverseDynamics(InverseDynamicsTest::GRAVITY_COMPENSATION);
		// testInverseDynamics(InverseDynamicsTest::NON_LINEAR_EFFECTS);

		// testJacobians();

		// testGetDynamicInfo();
		testKinematicInfo();
		// testJointLimits();
		// Send dds message
		ddslink->sendMessage("gazebo_glue_code_test", &msg);
	}

	void GazeboPluginGlueTest::testForwardKinematics(){
		const std::string base_frame = "base_link";
		auto base_pose = this->sim_model->GetLink(base_frame)->WorldPose();
		int i = 0;
		for(auto &leg : *pRobot->getLegs())
		{
			std::string leg_name = leg->getName();
			std::string leg_name_lower = leg_name;
			std::transform(leg_name_lower.begin(), leg_name_lower.end(), leg_name_lower.begin(), ::tolower);
			const std::string frame = leg_name_lower+"_foot";
			auto frame_pose = this->sim_model->GetLink(frame)->WorldPose();
			auto frame_vel = this->sim_model->GetLink(frame)->WorldLinearVel();
			auto b_frame_pose = base_pose.Inverse() * frame_pose;
			auto b_frame_vel = base_pose.Inverse().Rot() * frame_vel;
			auto ee_position = pRobot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
			auto ee_velocity = pRobot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
			pRobot->forwardKinematics(*joints_positions, *joints_velocity, ee_position, ee_velocity);
			// -- fill dds message field
			// --- position and velocity
			msg.feet_pos_gt()[i+0] = b_frame_pose.Pos().X();
			msg.feet_pos_gt()[i+1] = b_frame_pose.Pos().Y();
			msg.feet_pos_gt()[i+2] = b_frame_pose.Pos().Z();
			msg.feet_vel_gt()[i+0] = b_frame_vel.X();
			msg.feet_vel_gt()[i+1] = b_frame_vel.Y();
			msg.feet_vel_gt()[i+2] = b_frame_vel.Z();
			msg.feet_pos()[i+0] = ee_position[leg_name](0);
			msg.feet_pos()[i+1] = ee_position[leg_name](1);
			msg.feet_pos()[i+2] = ee_position[leg_name](2);
			msg.feet_vel()[i+0] = ee_velocity[leg_name](0);
			msg.feet_vel()[i+1] = ee_velocity[leg_name](1);
			msg.feet_vel()[i+2] = ee_velocity[leg_name](2);
			i += leg->getJoints()->size();
		}
	}

	void GazeboPluginGlueTest::testGetPose(){
		// test getFoot position, getFoot orientation and getFoot pose
		// get relative position of left lower leg w.r.t right upper leg
		const std::string from_frame = "lf_lowerleg";
		const std::string to_frame = "rf_upperleg";
		auto from_pose = this->sim_model->GetLink(from_frame)->WorldPose();
		auto to_pose = this->sim_model->GetLink(to_frame)->WorldPose();
		auto relative_pose_gt = to_pose.Inverse() * from_pose;

		// -- fill dds message field
		// --- position
		msg.relative_pos_gt()[0] = relative_pose_gt.Pos().X();
		msg.relative_pos_gt()[1] = relative_pose_gt.Pos().Y();
		msg.relative_pos_gt()[2] = relative_pose_gt.Pos().Z();

		// --- orientation (Z, Y, X euler angles)
		msg.relative_ori_gt()[0] = relative_pose_gt.Rot().Euler()[0];
		msg.relative_ori_gt()[1] = relative_pose_gt.Rot().Euler()[1];
		msg.relative_ori_gt()[2] = relative_pose_gt.Rot().Euler()[2];

		// compute relative pose using robotlib

		// -- call robotlib getFramePosition and getFrameOrientation
		robotlib::LimbDataMap<Eigen::Vector3d> end_effector_position =  pRobot->makeLimbDataMap<Eigen::Vector3d>();

		auto relative_pos = pRobot->getFramePosition(*joints_positions, from_frame, to_frame);
		auto relative_rot = pRobot->getFrameOrientation(*joints_positions, from_frame, to_frame);
		auto relative_ori = dls::math::rotTorpy(relative_rot.transpose());

		// -- fill dds message field
		// --- position
		msg.relative_pos()[0] = relative_pos(0);
		msg.relative_pos()[1] = relative_pos(1);
		msg.relative_pos()[2] = relative_pos(2);
		// --- orientation (Z, Y, X euler angles)
		msg.relative_ori()[0] = relative_ori(0);
		msg.relative_ori()[1] = relative_ori(1);
		msg.relative_ori()[2] = relative_ori(2);

		// -- call robotlib getFramePose
		auto relative_pose = pRobot->getFramePose(*joints_positions, from_frame, to_frame);
		relative_ori = dls::math::rotTorpy(relative_pose.block<3,3>(0,0).transpose());
		// -- fill dds message field
		msg.relative_pose()[0] = relative_pose.block<3,1>(0,3)(0);
		msg.relative_pose()[1] = relative_pose.block<3,1>(0,3)(1);
		msg.relative_pose()[2] = relative_pose.block<3,1>(0,3)(2);
		msg.relative_pose()[3] = relative_ori(0);
		msg.relative_pose()[4] = relative_ori(1);
		msg.relative_pose()[5] = relative_ori(2);
	}


	void GazeboPluginGlueTest::testInverseDynamics(const InverseDynamicsTest test_type){
		if(test_type==InverseDynamicsTest::GRAVITY_COMPENSATION)
		{
			testGravityCompensation();
		}
		else if (test_type==InverseDynamicsTest::NON_LINEAR_EFFECTS)
		{
			testNonLinearEffects();
		}
	}

	void GazeboPluginGlueTest::testGravityCompensation(){
		


		// define inputs: q, qd=qdd=0
		const std::string base_frame = "base_link";
		auto base_pose = this->sim_model->GetLink(base_frame)->WorldPose();
		auto w_base_lin_vel = this->sim_model->GetLink(base_frame)->WorldLinearVel();
		auto w_base_ang_vel = this->sim_model->GetLink(base_frame)->WorldAngularVel();
		auto b_base_lin_vel = base_pose.Rot().Inverse() * w_base_lin_vel;
		auto b_base_ang_vel = base_pose.Rot().Inverse() * w_base_ang_vel;

		Eigen::VectorXd q = pinocchio::neutral(robot_model);
		q.block<3,1>(0,0) = Eigen::Vector3d(base_pose.Pos().X(), base_pose.Pos().Y(), base_pose.Pos().Z());

		Eigen::Quaterniond quaternion_base = dls::math::rpyToquat(Eigen::Vector3d(base_pose.Rot().Euler()[0], base_pose.Rot().Euler()[1], base_pose.Rot().Euler()[2]));
		q(3) = quaternion_base.x();
		q(4) = quaternion_base.y();
		q(5) = quaternion_base.z();
		q(6) = quaternion_base.w();

		q.tail(pRobot->getNJOINTS()) = reorderJoints(joints_positions->vec_());
		Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
		Eigen::VectorXd qdd = Eigen::VectorXd::Zero(robot_model.nv);

		// 		auto frame_id = robot_model.getFrameId("lf_kfe_joint");
		// std::cout <<"***************************" << std::endl;
		// Eigen::MatrixXd footJac = Eigen::MatrixXd::Zero(6, robot_model.nv);
		// pinocchio::computeFrameJacobian(robot_model, robot_data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, footJac);
		// std::cout << footJac << std::endl;
		// std::cout << "############################" << std::endl;
		// compute base and joints gravity terms
		const int n_joints = pRobot->getNJOINTS();
		// -- method 1: using rnea
		// pinocchio::rnea(robot_model, robot_data, q, qd, qdd);
		// auto g_base = robot_data.tau.block<6,1>(0,0);
		// auto g_joints = robot_data.tau.tail(n_joints);
		// -- method 2: using computeGeneralizedGravity
		// pinocchio::computeGeneralizedGravity(robot_model, robot_data, q);
		// auto g_base = robot_data.g.block<6,1>(0,0);
		// auto g_joints = robot_data.g.tail(n_joints);
		// -- method 3: using robotlib
		Eigen::Matrix<double, 6, 1> g_base;
		robotlib::JointState g_joints = pRobot->makeJointState(0.0);
		pRobot->computeGravityTerm(q.block<7,1>(0,0), *joints_positions, g_base, g_joints);

		// find contacts and compute contact jacobian
		std::array<std::string,4> contact_frames = {"lf_foot", "rf_foot", "lh_foot", "rh_foot"};
		std::array<int, 4> contact_idx = {0, 0, 0, 0};
		int n_contacts = contact_frames.size();
		Eigen::MatrixXd J_linear_contacts_base = Eigen::MatrixXd::Zero(3*n_contacts, 6);
		Eigen::MatrixXd J_linear_contacts_joints = Eigen::MatrixXd::Zero(3*n_contacts, n_joints);
		for (int i=0; i<contact_frames.size(); i++)
		{
			contact_idx[i] = robot_model.getFrameId(contact_frames[i]);
			// compute contact jacobian
			Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
			pinocchio::computeFrameJacobian(robot_model, robot_data, q, contact_idx[i], pinocchio::LOCAL, J);
			J_linear_contacts_base.block<3, 6>(i*3, 0) = J.block<3, 6>(0, 0);
			J_linear_contacts_joints.block(i*3, 0, 3, n_joints) = J.block(0, 6, 3, n_joints);
		}
		Eigen::MatrixXd J_linear_contacts_base_T = J_linear_contacts_base.transpose();
		Eigen::MatrixXd J_linear_contacts_base_pseudo =  J_linear_contacts_base_T.completeOrthogonalDecomposition().pseudoInverse();

		// Contact forces at local coordinates (at each foot coordinate)
		Eigen::MatrixXd contact_f_des = J_linear_contacts_base_pseudo * g_base;
		// compute desired tau
		Eigen::MatrixXd J_linear_contacts_joints_T = J_linear_contacts_joints.transpose();
		Eigen::VectorXd tau = g_joints.toeig_() - J_linear_contacts_joints_T*contact_f_des;
		
		// ********************** Compute torques using rnea ********************** 
		pinocchio::framesForwardKinematics(robot_model, robot_data, q);

		// -- contact forces at the parent joint frame of the link in contact
		std::array<int, 4> joint_contact_idx = {0, 0, 0, 0};
		for(int i=0; i<n_contacts; i++)
		{
			joint_contact_idx[i] = robot_model.frames[contact_idx[i]].parentJoint;
		}
		pinocchio::container::aligned_vector<pinocchio::Force> joint_f_des(robot_model.njoints, pinocchio::Force::Zero());
		for(int i=0; i<n_contacts; i++)
		{
			pinocchio::Force contact_f_des_i(contact_f_des.block<3,1>(i*3, 0), Eigen::Vector3d::Zero());
			joint_f_des[joint_contact_idx[i]] = robot_data.oMi[joint_contact_idx[i]].actInv(
											robot_data.oMf[contact_idx[i]].act(contact_f_des_i));
		}
		// -- call rnea algorithm
		// method 1: call rnea
		pinocchio::rnea(robot_model, robot_data, q, qd, qdd, joint_f_des);
		// method 2: call computeStaticTorque
		// pinocchio::computeStaticTorque(robot_model, robot_data, q, joint_f_des);
		Eigen::VectorXd tau_rnea = robot_data.tau;
		// method 3: use robotlib
		robotlib::JointState tau_joints = pRobot->makeJointState(0.0);
		robotlib::LimbDataMap<Eigen::Vector3d> ee_position = pRobot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero());
		pRobot->forwardKinematics(*joints_positions, ee_position);
		robotlib::eigen::aligned_map<std::string, Eigen::Vector3d> f_ext;
		for(int i=0; i<n_contacts; i++)
		{
			f_ext.insert({contact_frames[i], contact_f_des.block<3,1>(i*3, 0)});
		}
		pRobot->inverseDynamics(q.block<7,1>(0,0),
								Eigen::Matrix<double, 6, 1>::Zero(),
                                Eigen::Matrix<double, 6, 1>::Zero(),
                                *joints_positions,
                                pRobot->makeJointState(0.0),
                                pRobot->makeJointState(0.0),
                                f_ext,
                                tau_joints);
		

		// Save contact forces at base link frame for testing (q relative to base are in base coordinates?)	
		const int base_idx = robot_model.getFrameId(base_frame);
		Eigen::MatrixXd base_f_des = Eigen::MatrixXd::Zero(3*n_contacts, 1);
		for(int i=0; i<n_contacts; i++)
		{
			pinocchio::Force contact_f_des_i(contact_f_des.block<3,1>(i*3, 0),Eigen::Vector3d::Zero());
			base_f_des.block<3,1>(i*3, 0) = robot_data.oMf[base_idx].actInv(
											robot_data.oMf[contact_idx[i]].act(contact_f_des_i)).linear();
		}	

		// -- fill dds message field
		// --- forces
		for(int i=0; i<base_f_des.size(); i++)
		{
			msg.feet_forces()[i] = base_f_des(i);
		}
		// --- torques
		tau = reorderJoints(tau);
		// for(int i=0; i<n_joints; i++)
		// {
		// 	msg.tau()[i+6] = tau(i);
		// }
		for(int i=0; i<n_joints; i++)
		{
			msg.tau()[i+6] = tau_joints.toeig_()(i); // robotlib
		}
		for(int i = 0; i< n_joints; i++){
			msg.tau_gt()[i+6] = joints_torques->vec_()(i);
		}
		// tau_rnea.tail(pRobot->getNJOINTS()) = reorderJoints(tau_rnea.tail(pRobot->getNJOINTS()));
		// for(int i = 0; i< robot_model.nv; i++){
		// 	msg.tau_gt()[i] = tau_rnea(i);
		// }
	}

	void GazeboPluginGlueTest::testNonLinearEffects(){
		const std::string base_frame = "base_link";
		auto base_pose = this->sim_model->GetLink(base_frame)->WorldPose();
		// robot state
		Eigen::VectorXd q = pinocchio::neutral(robot_model);
		q.block<3,1>(0,0) = Eigen::Vector3d(base_pose.Pos().X(), base_pose.Pos().Y(), base_pose.Pos().Z());
		Eigen::Quaterniond quaternion_base = dls::math::rpyToquat(Eigen::Vector3d(base_pose.Rot().Euler()[0], base_pose.Rot().Euler()[1], base_pose.Rot().Euler()[2]));
		q(3) = quaternion_base.x();
		q(4) = quaternion_base.y();
		q(5) = quaternion_base.z();
		q(6) = quaternion_base.w();
		q.tail(pRobot->getNJOINTS()) = reorderJoints(joints_positions->vec_());

		// robot velocity
		Eigen::VectorXd qd = Eigen::VectorXd::Zero(robot_model.nv);
		auto w_base_lin_vel = this->sim_model->GetLink(base_frame)->WorldLinearVel();
		auto w_base_ang_vel = this->sim_model->GetLink(base_frame)->WorldAngularVel();
		auto b_base_lin_vel = base_pose.Rot().Inverse() * w_base_lin_vel;
		auto b_base_ang_vel = base_pose.Rot().Inverse() * w_base_ang_vel;
		qd.head(3) = Eigen::Vector3d(b_base_lin_vel.X(), b_base_lin_vel.Y(), b_base_lin_vel.Z());
		qd.segment(3,3) = Eigen::Vector3d(b_base_ang_vel.X(), b_base_ang_vel.Y(), b_base_ang_vel.Z());
		qd.tail(pRobot->getNJOINTS()) = reorderJoints(joints_velocity->vec_());
		
		// non linear effects
		Eigen::VectorXd nle = Eigen::VectorXd::Zero(robot_model.nv);
		// method 1: using pinocchio
		// pinocchio::nonLinearEffects(robot_model, robot_data, q, qd);
		// nle = robot_data.nle;
		// nle.tail(pRobot->getNJOINTS()) = reorderJoints(nle.tail(pRobot->getNJOINTS()));
		// method 2: using robotlib
		Eigen::Matrix<double, 6, 1> nle_base;
		auto nle_joints = pRobot->makeJointState(0.0);
		Eigen::Matrix<double, 6, 1> base_vel = qd.head(6);
		pRobot->computeNonLinearEffects(q.block<7,1>(0,0), base_vel, *joints_positions, *joints_velocity, nle_base, nle_joints);
		nle.head(6) = nle_base;
		nle.tail(pRobot->getNJOINTS()) = nle_joints.toeig_();
		
		// non linear effects without base velocity
		// method 1: using pinocchio
		// qd.head(3) = Eigen::Vector3d::Zero();
		// qd.segment(3,3) = Eigen::Vector3d::Zero();
		// pinocchio::nonLinearEffects(robot_model, robot_data, q, qd);
		// auto nle_no_base_info = robot_data.nle;
		// nle_no_base_info.tail(pRobot->getNJOINTS()) = reorderJoints(nle_no_base_info.tail(pRobot->getNJOINTS()));
		// method 2: using robotlib
		Eigen::VectorXd nle_no_base_info = Eigen::VectorXd::Zero(robot_model.nv);
		pRobot->computeNonLinearEffects(q.block<7,1>(0,0), Eigen::Matrix<double, 6, 1>::Zero(), *joints_positions, *joints_velocity, nle_base, nle_joints);
		nle_no_base_info.head(6) = nle_base;
		nle_no_base_info.tail(pRobot->getNJOINTS()) = nle_joints.toeig_();

		// Compute non linear effect without considering base velocity and getting only the nle acting on the joints
		// pRobot->computeNonLinearEffects(q.block<7,1>(0,0), *joints_positions, *joints_velocity, nle_joints);

		// -- fill dds message field
		for(int i = 0; i< robot_model.nv; i++){
			msg.nle()[i] = nle(i);
			msg.nle_no_base_info()[i] = nle_no_base_info(i);
		}
	}

	void GazeboPluginGlueTest::testJacobians(){
	// obj: get the jacobian in base frame considering the fact that the robot has a floating base
	// set frame
	const std::string frame = "rf_foot";	
	// frame pose
	auto w_frame_pose = this->sim_model->GetLink(frame)->WorldPose();
	// frame velocity in world frame
	auto w_frame_vel = this->sim_model->GetLink(frame)->WorldLinearVel();
	auto w_frame_ang_vel = this->sim_model->GetLink(frame)->WorldAngularVel();
	// base velocity in world frame
	auto w_base_vel = this->sim_model->GetLink(base_frame)->WorldLinearVel();
	auto w_base_ang_vel = this->sim_model->GetLink(base_frame)->WorldAngularVel();
	// base velocity in base frame
	auto b_base_vel = base_pose.Rot().Inverse() * w_base_vel;
	auto b_base_ang_vel = base_pose.Rot().Inverse() * w_base_ang_vel;
	// frame velocity in base frame
	auto b_base_to_frame_pos = base_pose.Inverse().Rot()*(w_frame_pose.Pos() - base_pose.Pos());
	auto b_frame_vel = base_pose.Inverse().Rot() * w_frame_vel - b_base_vel - b_base_ang_vel.Cross(b_base_to_frame_pos);
	auto b_frame_ang_vel = base_pose.Inverse().Rot() * w_frame_ang_vel- b_base_ang_vel;

	// ********* compute foot velocity using pinocchio**********
	// getFrameJacobian with LOCAL_WORLD_ALIGNED
	auto frame_id = robot_model.getFrameId(frame);
	Eigen::MatrixXd footJac = Eigen::MatrixXd::Zero(6, robot_model.nv);
	pinocchio::computeFrameJacobian(robot_model, robot_data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, footJac);

	// compute linear foot velocity in world frame
	Eigen::MatrixXd footJac_lin = footJac.block(0,0,3, robot_model.nv);
	Eigen::Vector3d w_frame_vel_pin = footJac_lin*qd;
	// compute linear foot velocity in base frame
	footJac_lin.block<3,6>(0,0).setZero();
	Eigen::Matrix3d b_R_w = dls::math::quatToRotMat(Eigen::Quaterniond(q.block<4,1>(3,0))); // orientation of the world frame expressed in base frame
	Eigen::MatrixXd b_footJac_lin = b_R_w*footJac_lin;
	Eigen::Vector3d b_frame_vel_pin = b_footJac_lin*qd;


	// compute angular velocity in world frame
	Eigen::MatrixXd footJac_ang = footJac.block(3,0,3, robot_model.nv);
	Eigen::Vector3d w_frame_ang_vel_pin = footJac_ang*qd;
	// compute angular velocity in base frame
	footJac_ang.block<3,6>(0,0).setZero();
	Eigen::MatrixXd b_footJac_ang = b_R_w*footJac_ang;
	Eigen::Vector3d b_frame_ang_vel_pin = b_footJac_ang*qd;
	
	// ********* compute foot velocity using robotlib**********
	// -- method 1: using limbs jacobian
	// Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, pRobot->getNJOINTS());
	// pRobot->computeLimbsJacobian(*joints_positions, frame, J);
	// qd_ordered.tail(pRobot->getNJOINTS()) = reorderJoints(qd.tail(pRobot->getNJOINTS()));
	// b_frame_vel_pin = J.block(0,0,3, pRobot->getNJOINTS()) *qd_ordered.tail(pRobot->getNJOINTS());
	// b_frame_ang_vel_pin = J.block(3,0,3, pRobot->getNJOINTS()) *qd_ordered.tail(pRobot->getNJOINTS());

	// -- method 2: using whole body jacobian
	Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot_model.nv);
	pRobot->computeWholeBodyJacobian(q.head(7), *joints_positions, frame, J);
	J.block<6,6>(0,0).setZero();
	auto qd_ordered = qd;
	qd_ordered.tail(pRobot->getNJOINTS()) = reorderJoints(qd.tail(pRobot->getNJOINTS()));
	b_frame_vel_pin = J.block(0,0,3, robot_model.nv ) * qd_ordered;
	b_frame_ang_vel_pin = J.block(3,0,3, robot_model.nv) * qd_ordered;

	// -- fill dds message field
	msg.feet_vel()[0] = w_frame_vel_pin(0);
	msg.feet_vel()[1] = w_frame_vel_pin(1);
	msg.feet_vel()[2] = w_frame_vel_pin(2);
	msg.feet_vel_gt()[0] = w_frame_vel.X();
	msg.feet_vel_gt()[1] = w_frame_vel.Y();
	msg.feet_vel_gt()[2] = w_frame_vel.Z();
	msg.feet_vel()[3] = b_frame_vel_pin(0);
	msg.feet_vel()[4] = b_frame_vel_pin(1);
	msg.feet_vel()[5] = b_frame_vel_pin(2);
	msg.feet_vel_gt()[3] = b_frame_vel.X();
	msg.feet_vel_gt()[4] = b_frame_vel.Y();
	msg.feet_vel_gt()[5] = b_frame_vel.Z();
	msg.feet_vel()[6] = w_frame_ang_vel_pin(0);
	msg.feet_vel()[7] = w_frame_ang_vel_pin(1);
	msg.feet_vel()[8] = w_frame_ang_vel_pin(2);
	msg.feet_vel_gt()[6] = w_frame_ang_vel.X();
	msg.feet_vel_gt()[7] = w_frame_ang_vel.Y();
	msg.feet_vel_gt()[8] = w_frame_ang_vel.Z();

	msg.feet_vel()[9] = b_frame_ang_vel_pin(0);
	msg.feet_vel()[10] = b_frame_ang_vel_pin(1);
	msg.feet_vel()[11] = b_frame_ang_vel_pin(2);
	msg.feet_vel_gt()[9] = b_frame_ang_vel.X();
	msg.feet_vel_gt()[10] = b_frame_ang_vel.Y();
	msg.feet_vel_gt()[11] = b_frame_ang_vel.Z();

	}
	Eigen::VectorXd GazeboPluginGlueTest::reorderJoints(const Eigen::VectorXd& data) const{
        Eigen::VectorXd new_data = data;
        for(auto &[key, value] : idx_map)
        {
            new_data[value] = data[key];
            new_data[key] = data[value];
        }
        return new_data;
    }

	void GazeboPluginGlueTest::testGetDynamicInfo(){
		std::cout << "Robot total mass " << pRobot->getRobotMass() << std::endl;
		std::cout << "Robot total com " << pRobot->computeWholeBodyCoM(*joints_positions).transpose() << std::endl;
		// std::cout << "Robot total inertia " << pRobot->getRobotInertia() << std::endl;
		std::string link_name = "rh_upperleg";
		std::cout << "Mass of link " << link_name << " is " << pRobot->getLinkMass(link_name) << std::endl;
        pinocchio::computeSubtreeMasses(robot_model, robot_data);
		int joint_id = robot_model.getJointId("lf_haa_joint");
		std::cout << "LF leg mass " << robot_data.mass[joint_id] << std::endl;
        std::cout <<"Get Link intertia (lf_lowerleg): " <<  pRobot->getLinkInertia("lf_lowerleg") << std::endl;
		std::cout << "Get Link CoM (lf_lowerleg): " << pRobot->getLinkCOM("lf_lowerleg") << std::endl;
		std::cout << "computeCoMFromBase: " << pRobot->computeCoMFromBase(*joints_positions, q.head(7)).transpose();
		std::cout << "computeBaseFromCoM: " << pRobot->computeBaseFromCoM(*joints_positions, q.block<4,1>(3,0), pRobot->getCoMFromBase(*joints_positions, q.head(7))).transpose() << ", base position: " << q.head(3).transpose()<< std::endl;
	}

	void GazeboPluginGlueTest::testJointLimits(){
		for(auto leg: *this->pRobot->getLegs())
		{
			for(auto joint : *leg->getJoints())
			{
				const std::string joint_name = joint->getName();
				std::cout << "q_min for "<< joint_name << ": "<<pRobot->getMinJointAngle(joint) << std::endl;
				std::cout << "q_max for "<< joint_name << ": "<<pRobot->getMaxJointAngle(joint) << std::endl;
				std::cout << "qd_max for "<< joint_name << ": "<<pRobot->getMaxJointVelocity(joint) << std::endl;
				std::cout << "tau_max for "<< joint_name << ": "<<pRobot->getMaxJointEffort(joint) << std::endl;
			}
			std::cout << "****************\n";
		}
		std::cout << "#############################\n";
	}

	void GazeboPluginGlueTest::testKinematicInfo(){
		std::cout << pRobot->getImuBaseOffset() << std::endl;
	}
}