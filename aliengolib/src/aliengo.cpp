#include "aliengo.hpp"
#include "aliengo_leg.hpp"
#include "urdf_params_getter.h"
#include <urdf/model.h>

namespace aliengolib
{

    Aliengo::Aliengo(const std::shared_ptr<robotlib::Trunk> trunk,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs,
                const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms)
        : Robot<NJOINTS_TOT, NLINKS_TOT, NLEGS, NARMS>(
                "Aliengo",
                trunk,
                std::make_shared<const robotlib::Container<std::shared_ptr<robotlib::LimbBase>, NLEGS>>(legs),
                std::make_shared<const robotlib::Container<std::shared_ptr<robotlib::LimbBase>, NARMS>>(arms)),
                kinConfig_(this->makeLegDataMap<KinematicsConfig>()),
                b_R_h_(this->makeLegDataMap<Eigen::Matrix<double, 3, 3>>()),
                h_R_b_(this->makeLegDataMap<Eigen::Matrix<double, 3, 3>>()),
                hipPos_(this->makeLegDataMap<Eigen::Matrix<double, 3, 1>>())                
                // TODO: robcogen
                // invdyn_(rcg::InverseDynamics(inertias, transforms)),
                // jacobians_(rcg::Jacobians())
    {
        std::array<std::shared_ptr<robotlib::Joint>, NLEGS> children;

        children[0] = getJoint("LF_HAA");
        children[1] = getJoint("RF_HAA");
        children[2] = getJoint("LH_HAA");
        children[3] = getJoint("RH_HAA");

        setChildrenOfTrunk(std::make_shared<robotlib::Container<std::shared_ptr<robotlib::Joint>, NLEGS>>(children));

        setParentOfLink(trunk_, nullptr);

        for (auto leg : *(this->getLegs()))
        {
            for (auto joint : *(leg->getJoints()))
            {
                // const std::string child_name = leg->jointToChildName(joint);

                // setChildOfJoint(joint, getLink(child_name));
                // const std::string parent_name = leg->jointToParentName(joint);
                // setParentOfJoint(joint, getLink(parent_name));
            }
        }

        for (auto leg : *(this->getLegs()))
        {
            for (auto link : *(leg->getLinks()))
            {
                const std::string child_name = leg->linkToChildName(link);
                setChildOfLink(link, getJoint(child_name));

                const std::string parent_name = leg->linkToParentName(link);
                setParentOfLink(link, getJoint(parent_name));
            }
        }
        
        std::string robot_description{readURDFPugixml("../include/aliengo.urdf")};
        if(robot_description.compare(""))
            std::cout << "Failed to read the urdf using pugixml" << std::endl;

        urdf::Model robot_model;
        if (!robot_model.initString(robot_description))
            std::cout << "Failed to parse urdf file" << std::endl;

        // param_getter_.reset(new iit::dog::UrdfParamsGetter(robot_model));
        
        // Define kinematic parameters
        setKinematicsParameters();
    };

    
    Aliengo::~Aliengo(){};

    void Aliengo::setKinematicsParameters()
    {
        //Leg kinematic configurations
        kinConfig_["LF"].thorax_conf = 1;
        kinConfig_["LF"].thigh            = 0.22; //peripheral_lowerleg_x
        kinConfig_["LF"].thigh_horizontal = 0.0409+0.03931; //peripheral_lowerleg_z - peripheral_upperleg_y
        kinConfig_["LF"].thigh_vertical   = 0.0587;
        kinConfig_["LF"].knee_offset      = 0.00674; //peripheral_foot_z
        kinConfig_["LF"].base_offset      = 0.0;
        kinConfig_["LF"].shank            = 0.28;//peripheral_foot_x

        for (short i = 0; i < 4; ++i) {
            kinConfig_["LF"].min_joints[i]    = -M_PI;
            kinConfig_["LF"].max_joints[i]    = +M_PI;
        }

        //Leg kinematic configurations
        kinConfig_["RF"].thorax_conf = -1;
        kinConfig_["RF"].thigh            = 0.22; //peripheral_lowerleg_x
        kinConfig_["RF"].thigh_horizontal = 0.0409+0.03931; //peripheral_lowerleg_z - peripheral_upperleg_y
        kinConfig_["RF"].thigh_vertical   = 0.0587;
        kinConfig_["RF"].knee_offset      = 0.00674; //peripheral_foot_z
        kinConfig_["RF"].base_offset      = 0.0;
        kinConfig_["RF"].shank            = 0.28;//peripheral_foot_x

        for (short i = 0; i < 4; ++i) {
            kinConfig_["RF"].min_joints[i]    = -M_PI;
            kinConfig_["RF"].max_joints[i]    = +M_PI;
        }

        //Leg kinematic configurations
        kinConfig_["LH"].thorax_conf = 1;
        kinConfig_["LH"].thigh            = 0.22; //peripheral_lowerleg_x
        kinConfig_["LH"].thigh_horizontal = 0.0409+0.03931; //peripheral_lowerleg_z - peripheral_upperleg_y
        kinConfig_["LH"].thigh_vertical   = 0.0587;
        kinConfig_["LH"].knee_offset      = 0.00674; //peripheral_foot_z
        kinConfig_["LH"].base_offset      = 0.0;
        kinConfig_["LH"].shank            = 0.28;//peripheral_foot_x

        for (short i = 0; i < 4; ++i) {
            kinConfig_["LH"].min_joints[i]    = -M_PI;
            kinConfig_["LH"].max_joints[i]    = +M_PI;
        }

        //Leg kinematic configurations
        kinConfig_["RH"].thorax_conf = -1;
        kinConfig_["RH"].thigh            = 0.22; //peripheral_lowerleg_x
        kinConfig_["RH"].thigh_horizontal = 0.0409+0.03931; //peripheral_lowerleg_z - peripheral_upperleg_y
        kinConfig_["RH"].thigh_vertical   = 0.0587;
        kinConfig_["RH"].knee_offset      = 0.00674; //peripheral_foot_z
        kinConfig_["RH"].base_offset      = 0.0;
        kinConfig_["RH"].shank            = 0.28;//peripheral_foot_x

        for (short i = 0; i < 4; ++i) {
            kinConfig_["RH"].min_joints[i]    = -M_PI;
            kinConfig_["RH"].max_joints[i]    = +M_PI;
        }

        //Transformation matrix from the hip frame to the base frame
        b_R_h_["LF"].setZero();
        b_R_h_["LF"](0,0) = -1;
        b_R_h_["LF"](1,2) = +1;
        b_R_h_["LF"](2,1) = +1;

        b_R_h_["RF"].setZero();
        b_R_h_["RF"](0,0) = +1;
        b_R_h_["RF"](1,2) = -1;
        b_R_h_["RF"](2,1) = +1;

        b_R_h_["LH"] = b_R_h_["LF"];
        b_R_h_["RH"] =  b_R_h_["RF"];

        //Transformation matrix from the base frame to the hip frame
        h_R_b_["LF"].setZero();
        h_R_b_["LF"](0,0) = -1;
        h_R_b_["LF"](1,2) = +1;
        h_R_b_["LF"](2,1) = +1;

        h_R_b_["RF"].setZero();
        h_R_b_["RF"](0,0) = +1;
        h_R_b_["RF"](1,2) = +1;
        h_R_b_["RF"](2,1) = -1;

        h_R_b_["LH"] = h_R_b_["LF"];
        h_R_b_["RH"] = h_R_b_["RF"];

        //Hip position with respect to the base frame
        hipPos_["LF"] << +0.32729, +0.16972, -0.02722;
        hipPos_["RF"] << +0.32729, -0.16972, -0.02722;
        hipPos_["LH"] << -0.32729, +0.16972, -0.02722;
        hipPos_["RH"] << -0.32729, -0.16972, -0.02722;
    }

    Eigen::Vector3d Aliengo::hipToBasePosition(const Eigen::Vector3d& pos, const std::shared_ptr<robotlib::LimbBase> leg)
    {
        return (b_R_h_[leg] * pos + hipPos_[leg]);
    }

    Eigen::Vector3d Aliengo::baseToHipPosition(const Eigen::Vector3d& pos, const std::shared_ptr<robotlib::LimbBase> leg)
    {
        return h_R_b_[leg] * (pos - hipPos_[leg]);
    }

    void Aliengo::updateLinearJacobian(const robotlib::RobotBase::JointState &joints_positions,
                                    robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> &robot_jacobian)
    {
        Eigen::Matrix<double, 18, 1> joints_positions_matrix;
        int count{0};

        for (auto leg : *this->getLegs())
        {
            for(auto joint : *leg->getJoints())
			{
                joints_positions_matrix[count] = joints_positions[joint];
                count++;
            }
        }
        // TODO: robcogen
        // robot_jacobian["LF"].block<3,3>(0,0) = jacobians_.fr_trunk_J_lf_foot(joints_positions_matrix).block<3,3>(3,0);
		// robot_jacobian["RF"].block<3,3>(0,0) = jacobians_.fr_trunk_J_rf_foot(joints_positions_matrix).block<3,3>(3,0);
		// robot_jacobian["LH"].block<3,3>(0,0) = jacobians_.fr_trunk_J_lh_foot(joints_positions_matrix).block<3,3>(3,0);
		// robot_jacobian["RH"].block<3,3>(0,0) = jacobians_.fr_trunk_J_rh_foot(joints_positions_matrix).block<3,3>(3,0);
    }

    /// TODO: Robotlib - joint_velocity and joint_acceleration variables are not used
    void Aliengo::forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                            const robotlib::RobotBase::JointState &joint_velocity,
                            const robotlib::RobotBase::JointState &joint_acceleration,
                            robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                            robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                            robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration)
    {
        for (auto leg : *this->getLegs())
        {
            FK_In fkIn{};
            FK_Out fkOut{};
            //Load input variables
            auto legJointState = joint_position.getLegJointState(leg);
            
            // TOOD: remove hack - adjust FK_In struct to robotlib
            fkIn.cur_joints[0] = 0.0;
            int idx = 1;
            for (auto jointPair : *legJointState)
            {
                fkIn.cur_joints[idx] = jointPair.data_;
                idx++;
            }
            
            // TODO: robcogen
            // evaluate_forward_kinematics(&fkIn, &kinConfig_[leg], &fkOut);
            
            //Load output solution (solution in the hip frame)
            end_effector_position[leg](0) = fkOut.pos_x;
            end_effector_position[leg](1) = fkOut.pos_y;
            end_effector_position[leg](2) = fkOut.pos_z;

            //Transform solution to the base frame
            end_effector_position[leg] = hipToBasePosition(end_effector_position[leg], leg);

            //TODO
            end_effector_velocity[leg].setZero();
            end_effector_acceleration[leg].setZero();
        }
    }

    void Aliengo::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                const JointState &joint_position,
                                const JointState &joint_velocity,
                                const JointState &joint_acceleration,
                                Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                JointState &tau_joints)                   ///output
    {
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Aliengo!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;
        Eigen::Matrix<double, NJOINTS_TOT, 1> qd_robcogen;
        Eigen::Matrix<double, NJOINTS_TOT, 1> qdd_robcogen;
        Eigen::Matrix<double, NJOINTS_TOT, 1> invDynTau_robcogen;            
        int count = 0;
        for(auto leg : *this->getLegs())
        {
            for(auto joint : *leg->getJoints())
            {
                q_robcogen[count] = joint_position[joint];
                qd_robcogen[count] = joint_velocity[joint];    
                qdd_robcogen[count] = joint_acceleration[joint];   
                invDynTau_robcogen[count] = tau_joints[joint]; 
                count++;
            }
        }
        //TODO: robcogen
        //invdyn_.id_fully_actuated(wrench_base, invDynTau_robcogen,gravity_vector, robot_velocity, robot_acceleration,  q_robcogen, qd_robcogen, qdd_robcogen);
        // mapping of robcogen tau to robotlib tau
        count = 0;
        for(auto leg : *this->getLegs())
        {
            for(auto joint : *leg->getJoints())
            { 
                tau_joints[joint] = invDynTau_robcogen[count]; 
                count++;
            }
        }
    }

    void Aliengo::inverseKinematics(const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                 const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                 const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                 robotlib::RobotBase::JointState &joint_position,
                                 robotlib::RobotBase::JointState &joint_velocity,
                                 robotlib::RobotBase::JointState &joint_acceleration)
    {
        IK_In ikIn{};
        IK_Out ikOut{};

        for (auto leg : *this->getLegs())
        {
            /// Assigning inverse kinematics input parameters
            ikIn.pos_x = baseToHipPosition(end_effector_position[leg], leg)(0);
            ikIn.pos_y = baseToHipPosition(end_effector_position[leg], leg)(1);
            ikIn.pos_z = baseToHipPosition(end_effector_position[leg], leg)(2);

            /// Solving inverse kinematics
            evaluate_inverse_kinematics(&ikIn, &kinConfig_[leg], &ikOut);

            /// Assigning inverse kinematics outputs
            auto joint_haa = leg->getJoint(leg->getName() + "_HAA");
            auto joint_hfe = leg->getJoint(leg->getName() + "_HFE");
            auto joint_kfe = leg->getJoint(leg->getName() + "_KFE");
            
            joint_position[joint_haa] = ikOut.des_joints[1];
            joint_position[joint_hfe] = ikOut.des_joints[2];
            joint_position[joint_kfe] = ikOut.des_joints[3];
        }

        /// TODO
        joint_velocity.setZero();
        joint_acceleration.setZero();
    }

    void Aliengo::inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                 const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                 const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                 const robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> &robot_jacobian,
                                 JointState &joint_position,
                                 JointState &joint_velocity,
                                 JointState &joint_acceleration)
    {
        IK_In ikIn{};
        IK_Out ikOut{};

        for (auto leg : *this->getLegs())
        {
            /// Assigning inverse kinematics input parameters
            ikIn.pos_x = baseToHipPosition(end_effector_position[leg], leg)(0);
            ikIn.pos_y = baseToHipPosition(end_effector_position[leg], leg)(1);
            ikIn.pos_z = baseToHipPosition(end_effector_position[leg], leg)(2);

            /// Solving inverse kinematics
            evaluate_inverse_kinematics(&ikIn, &kinConfig_[leg], &ikOut);

            /// Assigning inverse kinematics outputs
            auto joint_haa = leg->getJoint(leg->getName() + "_HAA");
            auto joint_hfe = leg->getJoint(leg->getName() + "_HFE");
            auto joint_kfe = leg->getJoint(leg->getName() + "_KFE");

            joint_position[joint_haa] = ikOut.des_joints[1];
            joint_position[joint_hfe] = ikOut.des_joints[2];
            joint_position[joint_kfe] = ikOut.des_joints[3];

            ////Computing joint velocities from foot positions
            auto robot_jacobian_linear = robot_jacobian[leg].block<3,3>(0,0);
            auto leg_jacobian = robot_jacobian_linear.inverse() * end_effector_velocity[leg];

            joint_velocity[joint_haa] = leg_jacobian[0];
            joint_velocity[joint_hfe] = leg_jacobian[1];
            joint_velocity[joint_kfe] = leg_jacobian[2];
        }

        /// TODO
        joint_acceleration.setZero();
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

    extern "C" std::shared_ptr<robotlib::RobotBase> createRobot_t()
    {
        const std::shared_ptr<robotlib::Trunk> trunk = std::make_shared<robotlib::Trunk>("TRUNK");
        const std::array<std::shared_ptr<robotlib::LimbBase>, NLEGS> legs(
            {makeLeg("LF"),
             makeLeg("RF"),
             makeLeg("LH"),
             makeLeg("RH")});
        const std::array<std::shared_ptr<robotlib::LimbBase>, NARMS> arms({});

        return std::make_shared<Aliengo>(trunk, legs, arms);
    }
    extern "C" void destroyRobot_t(std::shared_ptr<robotlib::RobotBase> robot)
    {
    }
} // namespace hyqlib