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
                                         const std::shared_ptr<robotlib::Frame> origin,
                                         const std::shared_ptr<robotlib::Frame> destination) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            q_robcogen[joint_id] = q[joint];
        }

        return iit::rbd::Utils::positionVector( homogeneous_transforms_->getTransform(q_robcogen, glue_origin_frame_names_to_ids[origin->getName()], glue_destination_frame_names_to_ids[destination->getName()]) );
    }
    
    Eigen::Matrix3d Aliengo::getFrameOrientation(const robotlib::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> origin,
                                        const std::shared_ptr<robotlib::Frame> destination) const
    {
        q.size();
        origin->getName();
        destination->getName();

        std::cout << "TODO getFrameOrientation\n";

        return Eigen::Matrix3d().setZero();
    }

    Eigen::Matrix4d Aliengo::getFramePose(const robotlib::JointState &q,
                                    const std::shared_ptr<robotlib::Frame> origin,
                                    const std::shared_ptr<robotlib::Frame> destination) const
    {
        Eigen::Matrix4d frame_pose{};
        frame_pose.setZero();

        frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
        frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
        frame_pose.row(3) << 0, 0, 0, 1;

        return frame_pose;
    }

    Eigen::Vector3d Aliengo::getFootPosition(const robotlib::JointState &q,
                                    const std::shared_ptr<robotlib::Frame> foot) const
    {
        return this->getFramePosition(q, this->getLink("TRUNK"), foot);
    }

    Eigen::Matrix3d Aliengo::getFootOrientation(const robotlib::JointState &q,
                                        const std::shared_ptr<robotlib::Frame> foot) const
    {
        return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
    }

    Eigen::Matrix4d Aliengo::getFootPose(const robotlib::JointState &q,
                                const std::shared_ptr<robotlib::Frame> foot) const
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }

    Eigen::Vector3d Aliengo::getFootPosition(const robotlib::JointState &q,
                            const std::shared_ptr<robotlib::LimbBase> leg) const
    {
        return this->getFramePosition(q, this->getLink("TRUNK"), leg->getEndEffector());
    }

    Eigen::Matrix3d Aliengo::getFootOrientation(const robotlib::JointState &q,
                                        const std::shared_ptr<robotlib::LimbBase> leg) const
    {
        return this->getFrameOrientation(q, this->getLink("TRUNK"), leg->getEndEffector());
    }

    Eigen::Matrix4d Aliengo::getFootPose(const robotlib::JointState &q,
                                const std::shared_ptr<robotlib::LimbBase> leg) const
    {
        Eigen::Matrix4d foot_pose{};
        foot_pose.setZero();

        foot_pose.block(0, 3, 3, 1) << getFootPosition(q, leg->getEndEffector());
        foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, leg->getEndEffector());
        foot_pose.row(3) << 0, 0, 0, 1;

        return foot_pose;
    }
    
void Aliengo::updateLinearJacobian(const robotlib::JointState &joints_positions,
                                       robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const
{
    Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
    int count{0};

    for(auto joint : auxiliar_joints_variable_)
    {
        joints_positions_matrix[count] = joints_positions[joint];
        count++;
    }

    jacobians_->updateParameters();

    robot_jacobian["LF"].block<3,3>(0,0) = jacobians_->fr_trunk_J_LF_foot(joints_positions_matrix).block<3,3>(3,0);
	robot_jacobian["RF"].block<3,3>(0,0) = jacobians_->fr_trunk_J_RF_foot(joints_positions_matrix).block<3,3>(3,0);
	robot_jacobian["LH"].block<3,3>(0,0) = jacobians_->fr_trunk_J_LH_foot(joints_positions_matrix).block<3,3>(3,0);
	robot_jacobian["RH"].block<3,3>(0,0) = jacobians_->fr_trunk_J_RH_foot(joints_positions_matrix).block<3,3>(3,0);
}

void Aliengo::getFootJacobian(const robotlib::JointState &q,
                              const std::shared_ptr<robotlib::LimbBase> leg,
                              robotlib::Jacobian &footJac) const
{
    Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
    int count{0};

    for(auto joint : auxiliar_joints_variable_)
    {
        joints_positions_matrix[count] = q[joint];
        count++;
    }

    iit::dog::LegID leg_id = static_cast<iit::dog::LegID>(glue_leg_names_to_ids.at(leg->getName()));
    footJac.block<3,3>(0,0) = feet_jacobians_->getFootJacobian(joints_positions_matrix, leg_id);
    footJac.block<3,3>(3,0) = feet_jacobians_->getAngularFootJacobian(joints_positions_matrix, leg_id);
}


    void Aliengo::updateAngularJacobian(const robotlib::JointState &joints_positions,
                                       robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const
    {
        // TODO: test
        Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
        int count{0};

        for(auto joint : auxiliar_joints_variable_)
        {                
            joints_positions_matrix[count] = joints_positions[joint];
            count++;
        }

        jacobians_->updateParameters();
        robot_jacobian["LF"].block<3,3>(3,0) = jacobians_->fr_trunk_J_LF_foot(joints_positions_matrix).block<3,3>(0,0);
		robot_jacobian["RF"].block<3,3>(3,0) = jacobians_->fr_trunk_J_RF_foot(joints_positions_matrix).block<3,3>(0,0);
		robot_jacobian["LH"].block<3,3>(3,0) = jacobians_->fr_trunk_J_LH_foot(joints_positions_matrix).block<3,3>(0,0);
		robot_jacobian["RH"].block<3,3>(3,0) = jacobians_->fr_trunk_J_RH_foot(joints_positions_matrix).block<3,3>(0,0);
    }

    void Aliengo::updateLinearFootJacobian(const robotlib::JointState &joints_positions,
                                        const std::shared_ptr<robotlib::LimbBase> leg,
                                        robotlib::Jacobian &footJac) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
        int count{0};

        for(auto joint : auxiliar_joints_variable_)
        {
            joints_positions_matrix[count] = joints_positions[joint];
            count++;
        }

        jacobians_->updateParameters();
        if(leg->getName().compare("LF")==0)
        {
            footJac.block<3,3>(0,0) = jacobians_->fr_trunk_J_LF_foot(joints_positions_matrix).block<3,3>(3,0);
        }
        else if(leg->getName().compare("RF")==0)
        {
            footJac.block<3,3>(0,0) = jacobians_->fr_trunk_J_RF_foot(joints_positions_matrix).block<3,3>(3,0);
        }
        else if(leg->getName().compare("LH")==0)
        {
            footJac.block<3,3>(0,0) = jacobians_->fr_trunk_J_LH_foot(joints_positions_matrix).block<3,3>(3,0);
        }
        else if(leg->getName().compare("RH")==0)
        {
            footJac.block<3,3>(0,0) = jacobians_->fr_trunk_J_RH_foot(joints_positions_matrix).block<3,3>(3,0);
        }
    }

    void Aliengo::updateAngularFootJacobian(const robotlib::JointState &joints_positions,
                                        const std::shared_ptr<robotlib::LimbBase> leg,
                                        robotlib::Jacobian &footJac) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
        int count{0};

        for(auto joint : auxiliar_joints_variable_)
        {
            joints_positions_matrix[count] = joints_positions[joint];
            count++;
        }

        jacobians_->updateParameters();
        if(leg->getName().compare("LF"))
        {
            footJac.block<3,3>(3,0) = jacobians_->fr_trunk_J_LF_foot(joints_positions_matrix).block<3,3>(0,0);
        }
        else if(leg->getName().compare("RF"))
        {
            footJac.block<3,3>(3,0) = jacobians_->fr_trunk_J_RF_foot(joints_positions_matrix).block<3,3>(0,0);
        }
        else if(leg->getName().compare("LH"))
        {
            footJac.block<3,3>(3,0) = jacobians_->fr_trunk_J_LH_foot(joints_positions_matrix).block<3,3>(0,0);
        }
        else if(leg->getName().compare("RH"))
        {
            footJac.block<3,3>(3,0) = jacobians_->fr_trunk_J_RH_foot(joints_positions_matrix).block<3,3>(0,0);
        }
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position) const
    {
        homogeneous_transforms_->updateParameters();
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Aliengo!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;
        
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            q_robcogen[joint_id] = joint_position[joint];
        }
        end_effector_position["LF"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_LF_foot(q_robcogen));
        end_effector_position["RF"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_RF_foot(q_robcogen));
        end_effector_position["LH"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_LH_foot(q_robcogen));
        end_effector_position["RH"] = iit::rbd::Utils::positionVector( homogeneous_transforms_->fr_trunk_X_RH_foot(q_robcogen));
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    const robotlib::JointState &joint_velocity,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_velocity) const
    {
        forwardKinematics(joint_position, end_effector_position);

        for (auto leg : *legs_)
        {
            //TODO improve getting the jacobian from one leg only NRT!
            robotlib::LegDataMap<robotlib::Jacobian> full_feet_jacobian_tmp = this->makeFeetJacobian();
            updateLinearJacobian(joint_position, full_feet_jacobian_tmp);

            Eigen::Vector3d joint_velocity_leg{Eigen::Vector3d::Zero()};
            int count{0};
            for(auto joint: *leg->getJoints())
            {
                joint_velocity_leg[count] = joint_velocity[joint];
                count++;
            }

            end_effector_velocity[leg] =  full_feet_jacobian_tmp[leg].block<3,3>(0,0)*joint_velocity_leg;
        }
    }

    void Aliengo::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,    // robot base
                                const Eigen::Matrix<double, 6, 1> &robot_acceleration,  // robot base
                                const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                const robotlib::JointState &joint_acceleration,
                                Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                robotlib::JointState &tau_joints) const             ///output
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_velocity{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_acceleration{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};

        robcogen_joint_position.setZero();
        robcogen_joint_velocity.setZero();
        robcogen_joint_acceleration.setZero();
        robcogen_tau_joints.setZero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            robcogen_joint_position[joint_id] = joint_position[joint];
            robcogen_joint_velocity[joint_id] = joint_velocity[joint];
            robcogen_joint_acceleration[joint_id] = joint_acceleration[joint];
        }

        inverse_dynamics_->id_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robot_velocity, robot_acceleration, robcogen_joint_position, robcogen_joint_velocity, robcogen_joint_acceleration);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            tau_joints[joint] = robcogen_tau_joints[joint_id];
        }
    }

    void Aliengo::inverseDynamicsHTerm( robotlib::JointState &tau_joints,
                                        const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                        const robotlib::JointState &joint_position,
                                        const robotlib::JointState &joint_velocity,
                                        const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                        const Eigen::Matrix<double, 6, 1> &robot_acceleration) const
    {
        Eigen::Matrix<double, 6, 1> wrench_base(Eigen::Matrix<double, 6, 1>::Zero());
        inverseDynamics(robot_velocity, robot_acceleration, gravity_vector, joint_position, joint_velocity, this->makeJointState(0.0), wrench_base, tau_joints);
    }

    void Aliengo::computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                             const robotlib::JointState &joint_position,
                                             Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                             robotlib::JointState &tau_joints) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};

        robcogen_joint_position.setZero();
        robcogen_tau_joints.setZero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            robcogen_joint_position[joint_id] = joint_position[joint];
        }

        inverse_dynamics_->G_terms_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robcogen_joint_position);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            tau_joints[joint] = robcogen_tau_joints[joint_id];
        }
    }

    Eigen::Matrix<double, 6, 1> Aliengo::computeWrenchGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const robotlib::JointState &joint_position) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};
        Eigen::Matrix<double, 6, 1> wrench_base;

        robcogen_joint_position.setZero();
        robcogen_tau_joints.setZero();
        wrench_base.setZero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            robcogen_joint_position[joint_id] = joint_position[joint];
        }

        inverse_dynamics_->G_terms_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robcogen_joint_position);

        return wrench_base;
    }

    void Aliengo::computeTorquesGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                             const robotlib::JointState &joint_position,
                                             robotlib::JointState &tau_joints) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};
        Eigen::Matrix<double, 6, 1> wrench_base;

        robcogen_joint_position.setZero();
        robcogen_tau_joints.setZero();
        wrench_base.setZero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            robcogen_joint_position[joint_id] = joint_position[joint];
        }

        inverse_dynamics_->G_terms_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robcogen_joint_position);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids.at(joint->getName())};
            tau_joints[joint] = robcogen_tau_joints[joint_id];
        }
    }

     void Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                    robotlib::JointState &joint_position) const
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
                                    const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_acceleration,
                                    robotlib::JointState &joint_position,
                                    robotlib::JointState &joint_velocity,
                                    robotlib::JointState &joint_acceleration) const
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
        return inertias_->getTotalMass();
    }

    Eigen::Vector3d Aliengo::getRobotCoM() const {
        std::cout << "TODO: getRobotCoM" << std::endl;
        return Eigen::Vector3d().setZero();
    }

    Eigen::Matrix<double, 3, 1> Aliengo::getTrunkCOM() const
    {
        return inertias_->getCOM_trunk();
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