#include "aliengo.hpp"
#include "aliengo_leg.hpp"
#include "urdf_params_getter.h"

#include "robcogen/utils.h"

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
                const std::string child_name = leg->jointToChildName(joint);
                setChildOfJoint(joint, getLink(child_name));
                const std::string parent_name = leg->jointToParentName(joint);
                setParentOfJoint(joint, getLink(parent_name));
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

        if (!robot_model_.initString(robot_description))
            std::cout << "Failed to parse urdf file" << std::endl;

        // Getting joint limits from urdf
        
        
        param_getter_.reset(new iit::dog::UrdfParamsGetter(robot_model_));

        homogeneous_transforms_.reset(new iit::Aliengo::HomogeneousTransforms(*param_getter_));

        inverse_kinematics_.reset(new iit::Aliengo::InverseKinematics(*param_getter_));

        setJointLimitsFromUrdf();

        // ik_->setKinematicLimits(q_min_,q_max_);
        

    };

    
    Aliengo::~Aliengo(){};

    void Aliengo::setJointLimitsFromUrdf()
    {
        //Get limits from URDF for position, velocity and effort:
        for(std::pair<std::string, std::shared_ptr<urdf::Joint> > jointPair : robot_model_.joints_)
        {
            if (jointPair.second->type == urdf::Joint::REVOLUTE)
            {
                for (auto leg : *legs_)
                {
                    for (auto joint : *leg->getJoints())
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
                            
                            std::cout << "Set limits for joint " << urdf_joint_name << ":" << std::endl;
                            std::cout << "q_min = " << q_min << std::endl;
                            std::cout << "q_max = " << q_max << std::endl;
                            std::cout << "qd_max = " << qd_max << std::endl; 
                            std::cout << "tau_max = " << tau_max << std::endl;
                            std::cout << "\n";
                        }
                    }
                    
                }
            }
        }
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
        end_effector_position["RF"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_RF_foot(q_robcogen));
        end_effector_position["LH"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_LH_foot(q_robcogen));
        end_effector_position["RH"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_RH_foot(q_robcogen));
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

    Eigen::Matrix<double, 3, 1> Aliengo::getWholeBodyCOM()
    {
        std::cout << "TODO: glue code for getWholeBodyCOM()" << std::endl;
        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    Eigen::Matrix<double, 3, 1> Aliengo::getWholeBodyCOM(const JointState &joint_state)
    {
        std::cout << "TODO: glue code for getWholeBodyCOM(JS)" << std::endl;
        return Eigen::Matrix<double, 3, 1>::Zero();
    }

    Eigen::Vector3d Aliengo::getCoMFromBase(const JointState &q,
                                const Eigen::Vector3d &base_orient,
                                const Eigen::Vector3d &base_pos)
    {
        std::cout << "TODO: glue code for getCoMFromBase" << std::endl;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d Aliengo::getBaseFromCoM(const JointState &q,
                                const Eigen::Vector3d &base_orient,
                                const Eigen::Vector3d &CoM)
    {
        std::cout << "TODO: glue code for getBaseFromCoM" << std::endl;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVel(const JointState &q,
                                                const JointState &qd)
    {
        std::cout << "TODO: glue code for getWholeBodyCOMVel" << std::endl;
        return Eigen::Matrix<double, 6, 1>::Zero();
    }

    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &rotationMx,
                                                                const JointState &q,
                                                                const JointState &qd)
    {
        std::cout << "TODO: glue code for getWholeBodyCOMVelFB" << std::endl;
        return Eigen::Matrix<double, 6, 1>::Zero();
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