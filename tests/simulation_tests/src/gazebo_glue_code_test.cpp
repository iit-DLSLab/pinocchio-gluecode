#include "gazebo_glue_code_test.hpp"
#include <gazebo/sensors/sensors.hh>

#include <dls_messages/dds/gazebo_glue_code_testPubSubTypes.h>
#include <sstream>
#include <iostream>

namespace dls
{
	GazeboPluginGlueTest::GazeboPluginGlueTest() 
		: gazebo::ModelPlugin()
		, ddslink(std::make_shared<dls::DDSParticipant>(
			"GazeboPluginGlueTest::signals",
			dls::domains::signals
		))
	{
		ddslink->addWriter("gazebo_glue_code_test", dls::topicType("gazebo_glue_code_test", new GazeboGlueCodeTestMsgPubSubType()));
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
		
		this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin
		(
			std::bind(&GazeboPluginGlueTest::run, this)
		);

		std::cout << plugin_name << ": PLUGIN LOADED SUCCESSFULLY" << std::endl;
	}


    void GazeboPluginGlueTest::run()
	{
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
		// -- get joints data
		robotlib::JointState joints_positions = pRobot->makeJointState(0.0);
		robotlib::JointState joints_velocity = pRobot->makeJointState(0.0);
		robotlib::JointState joints_torques = pRobot->makeJointState(0.0);
		int i=0;
		for(auto &leg : joints_positions)
		{
			for(auto &joint : *leg.data_)
			{
				joints_positions[joint.key_] = this->sim_joints[i]->Position();
				joints_velocity[joint.key_] = this->sim_joints[i]->GetVelocity(0);
				joints_torques[joint.key_] = this->sim_joints[i]->GetForce(0);
				i++;
			}
		}
		// -- call robotlib getFramePosition and getFrameOrientation
		robotlib::LegDataMap<Eigen::Vector3d> end_effector_position =  pRobot->makeLegDataMap<Eigen::Vector3d>();

		auto relative_pos = pRobot->getFramePosition(joints_positions, from_frame, to_frame);
		auto relative_rot = pRobot->getFrameOrientation(joints_positions, from_frame, to_frame);
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
		auto relative_pose = pRobot->getFramePose(joints_positions, from_frame, to_frame);
		relative_ori = dls::math::rotTorpy(relative_pose.block<3,3>(0,0).transpose());
		// -- fill dds message field
		msg.relative_pose()[0] = relative_pose.block<3,1>(0,3)(0);
		msg.relative_pose()[1] = relative_pose.block<3,1>(0,3)(1);
		msg.relative_pose()[2] = relative_pose.block<3,1>(0,3)(2);
		msg.relative_pose()[3] = relative_ori(0);
		msg.relative_pose()[4] = relative_ori(1);
		msg.relative_pose()[5] = relative_ori(2);

		// Send dds message
		ddslink->sendMessage("gazebo_glue_code_test", &msg);
	}
	

}