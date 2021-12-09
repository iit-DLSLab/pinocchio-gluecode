#include "aliengo.hpp"
#include "aliengo_leg.hpp"
#include "urdf_params_getter.h"

#include "robcogen/utils.h"

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
        
        //TODO: improvements --> fix the path!
        std::string robot_description{readURDFifstream("/usr/include/urdf_robots/aliengo.urdf")};
        std::cout << robot_description ;
        urdf::Model robot_model;
        if (!robot_model.initString(robot_description))
            std::cout << "Failed to parse urdf file" << std::endl;

        param_getter_.reset(new iit::dog::UrdfParamsGetter(robot_model));

        homogeneous_transforms_.reset(new iit::Aliengo::HomogeneousTransforms(*param_getter_));

    };

    
    Aliengo::~Aliengo(){};

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
        homogeneous_transforms_->updateParameters();
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Crex!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;
        int count = 0;
        for(auto leg : *this->getLegs())
        {
            for(auto joint : *leg->getJoints())
            {
                q_robcogen[count] = joint_position[joint];
                count++;
            }
        }
        end_effector_position["LF"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_LF_foot(q_robcogen));


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

    }

    void Aliengo::inverseKinematics(const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                 const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                 const robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                 robotlib::RobotBase::JointState &joint_position,
                                 robotlib::RobotBase::JointState &joint_velocity,
                                 robotlib::RobotBase::JointState &joint_acceleration)
    {

    }

    void Aliengo::inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                 const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                 const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                 const robotlib::RobotBase::LegDataMap<robotlib::RobotBase::Jacobian> &robot_jacobian,
                                 JointState &joint_position,
                                 JointState &joint_velocity,
                                 JointState &joint_acceleration)
    {

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