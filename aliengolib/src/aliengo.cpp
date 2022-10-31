#include "aliengolib/aliengo.hpp"
#include "aliengolib/aliengo_leg.hpp"
#include "aliengolib/urdf_params_getter.h"
#include "aliengolib/robcogen/utils.h"

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
                const int joint_id{glue_joint_names_to_ids[joint->getName()]};
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

        // for (auto leg : *legs_)
        // {
        //     for(auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         RobotBase::getMinJointAngle(joint, robcogen_q_min[joint_id]);
        //         RobotBase::getMaxJointAngle(joint, robcogen_q_max[joint_id]);
        //     }
        // }
        
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            RobotBase::getMinJointAngle(joint, robcogen_q_min[joint_id]);
            RobotBase::getMaxJointAngle(joint, robcogen_q_max[joint_id]);
        }

        inverse_kinematics_->setKinematicLimits(robcogen_q_min, robcogen_q_max);
        inertias_.reset(new iit::Aliengo::dyn::InertiaProperties(*robot_params_));

        motion_transforms_.reset(new iit::Aliengo::MotionTransforms(*robot_params_));
        
        jacobians_.reset(new iit::Aliengo::Jacobians(*robot_params_));

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
    
    void Aliengo::updateLinearJacobian(const robotlib::JointState &joints_positions,
                                       robotlib::LegDataMap<robotlib::Jacobian> &robot_jacobian) const
    {
        // TODO: test
        Eigen::Matrix<double, NJOINTS_TOT, 1> joints_positions_matrix;
        int count{0};

        // for (auto leg : legs_)
        // {
        //     for(auto joint : *leg->getJoints())
		// 	{
        //         joints_positions_matrix[count] = joints_positions[joint];
        //         count++;
        //     }
        // }
        for(auto joint : auxiliar_joints_variable_)
        {                
            joints_positions_matrix[count] = joints_positions[joint];
            count++;
        }
        robot_jacobian["LF"].block<3,3>(0,0) = jacobians_->fr_trunk_J_LF_foot(joints_positions_matrix).block<3,3>(3,0);
		robot_jacobian["RF"].block<3,3>(0,0) = jacobians_->fr_trunk_J_RF_foot(joints_positions_matrix).block<3,3>(3,0);
		robot_jacobian["LH"].block<3,3>(0,0) = jacobians_->fr_trunk_J_LH_foot(joints_positions_matrix).block<3,3>(3,0);
		robot_jacobian["RH"].block<3,3>(0,0) = jacobians_->fr_trunk_J_RH_foot(joints_positions_matrix).block<3,3>(3,0);
    }

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                    robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position) const
    {
        homogeneous_transforms_->updateParameters();
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Crex!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;
        
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
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

    void Aliengo::forwardKinematics(const robotlib::JointState &joint_position,
                                const robotlib::JointState &joint_velocity,
                                const robotlib::JointState &joint_acceleration,
                                robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                robotlib::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) const
    {
        homogeneous_transforms_->updateParameters();
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Crex!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;

        for (auto leg : *this->getLegs())
        {
            for (auto joint : *leg->getJoints())
            {
                const int joint_id{glue_joint_names_to_ids[joint->getName()]};
                q_robcogen[joint_id] = joint_position[joint];
            }
        }

        end_effector_position["LF"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_LF_foot(q_robcogen));
        end_effector_position["RF"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_RF_foot(q_robcogen));
        end_effector_position["LH"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_LH_foot(q_robcogen));
        end_effector_position["RH"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_RH_foot(q_robcogen));

        for (auto leg : *this->getLegs())
        {
            // TODO improve getting the jacobian from one leg only NRT!
            robotlib::LegDataMap<robotlib::Jacobian> full_feet_jacobian_tmp = this->makeFeetJacobian();
            updateLinearJacobian(joint_position, full_feet_jacobian_tmp);

            Eigen::Vector3d joint_velocity_leg{Eigen::Vector3d::Zero()};
            int count{0};
            for (auto joint : *leg->getJoints())
            {
                joint_velocity_leg[count] = joint_velocity[joint];
                count++;
            }

            end_effector_velocity[leg] = full_feet_jacobian_tmp[leg].block<3, 3>(0, 0) * joint_velocity_leg;

            end_effector_acceleration[leg].setZero();
        }
    }

    robotlib::LegDataMap<Eigen::Vector3d> Aliengo::forwardKinematics(const robotlib::JointState &joint_position) const
    {
        homogeneous_transforms_->updateParameters();
    
        // Mapping from robotlib structure to robcogen ones, TODO: maybe a function mapping robotlib to eigen structure is needed
        // NB: this mapping assumes that the robcogen order is the same as the one defining the legs and joints of Crex!
        Eigen::Matrix<double, NJOINTS_TOT, 1> q_robcogen;

        for (auto leg : *this->getLegs())
        {
            for (auto joint : *leg->getJoints())
            {
                const int joint_id{glue_joint_names_to_ids[joint->getName()]};
                q_robcogen[joint_id] = joint_position[joint];
            }
        }

        robotlib::LegDataMap<Eigen::Vector3d> out(this->makeLegDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()));

        out["LF"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_LF_foot(q_robcogen));
        out["RF"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_RF_foot(q_robcogen));
        out["LH"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_LH_foot(q_robcogen));
        out["RH"] = iit::rbd::Utils::positionVector(homogeneous_transforms_->fr_trunk_X_RH_foot(q_robcogen));

        return out;        
    }


    void Aliengo::inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,    // robot base
                                  const Eigen::Matrix<double, 6, 1> &robot_acceleration,  // robot base
                                  const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                  const robotlib::JointState &joint_position,
                                  const robotlib::JointState &joint_velocity,
                                  const robotlib::JointState &joint_acceleration,
                                  Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                  robotlib::JointState &tau_joints) const   ///output
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_velocity{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_acceleration{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};

        robcogen_joint_position.setZero();
        robcogen_joint_velocity.setZero();
        robcogen_joint_acceleration.setZero();
        robcogen_tau_joints.setZero();
        
        // for (auto leg : *legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         robcogen_joint_position[joint_id] = joint_position[joint];
        //         robcogen_joint_velocity[joint_id] = joint_velocity[joint];
        //         robcogen_joint_acceleration[joint_id] = joint_acceleration[joint];
        //     }
        // }0

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            robcogen_joint_position[joint_id] = joint_position[joint];
            robcogen_joint_velocity[joint_id] = joint_velocity[joint];
            robcogen_joint_acceleration[joint_id] = joint_acceleration[joint];
        }

        inverse_dynamics_->id_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robot_velocity, robot_acceleration, robcogen_joint_position, robcogen_joint_velocity, robcogen_joint_acceleration);
        
        // for (auto leg : *legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         tau_joints[joint] = robcogen_tau_joints[joint_id];
        //     }
        // }

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            tau_joints[joint] = robcogen_tau_joints[joint_id];
        }
    }

    void Aliengo::computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                             const robotlib::JointState &joint_position,
                                             Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                             robotlib::JointState &tau_joints)              ///output
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_tau_joints{};

        robcogen_joint_position.setZero();
        robcogen_tau_joints.setZero();
        
        // for (auto leg : *legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         robcogen_joint_position[joint_id] = joint_position[joint];
        //     }
        // }

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            robcogen_joint_position[joint_id] = joint_position[joint];
        }

        inverse_dynamics_->G_terms_fully_actuated(wrench_base, robcogen_tau_joints, gravity_vector, robcogen_joint_position);
        
        // for (auto leg : *legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         tau_joints[joint] = robcogen_tau_joints[joint_id];
        //     }
        // }
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            tau_joints[joint] = robcogen_tau_joints[joint_id];
        }
    }

    void Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position,
                                    robotlib::JointState &joint_position) const
    {
        iit::dog::LegDataMap<Eigen::Vector3d> robcogen_end_effector_position{};
        Eigen::Matrix<double, NJOINTS_TOT, 1> robcogen_joint_position{};
        robcogen_joint_position.setZero();

        for (auto leg : *legs_)
        {
            const int leg_id{glue_leg_names_to_ids[leg->getName()]};

            robcogen_end_effector_position[leg_id] = end_effector_position[leg];
        }

        inverse_kinematics_->getJointPosition(robcogen_end_effector_position, robcogen_joint_position);

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
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
            const int leg_id{glue_leg_names_to_ids[leg->getName()]};

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
        
        // for (auto leg : *legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {   
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};

        //         joint_position[joint] = robcogen_joint_position[joint_id];
        //         joint_velocity[joint] = robcogen_joint_velocity[joint_id];
        //         joint_acceleration[joint] = robcogen_joint_acceleration[joint_id];
        //     }   
        // }

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            joint_position[joint] = robcogen_joint_position[joint_id];
            joint_velocity[joint] = robcogen_joint_velocity[joint_id];
            joint_acceleration[joint] = robcogen_joint_acceleration[joint_id];
        }
        // if(!iK_Check){
        //     des_q_ = q_;
        //     des_qd_ = qd_;
        //     des_qdd_ = JointState::Zero();
        //     referencesBackTracingPrintOuts(dog::LF);
        // }
    }

    robotlib::JointState Aliengo::inverseKinematics(const robotlib::LegDataMap<Eigen::Vector3d> &end_effector_position) const
    {
        iit::dog::FootPosition robcogen_end_effector_position{};
 
        iit::dog::LegJointState robcogen_joint_position{};
        robcogen_joint_position.setZero();

        robotlib::JointState out(this->makeJointState());
        
        Eigen::Array<bool, 3, 1> q_violation;
        for (auto leg : *legs_)
        {
            const int leg_id{glue_leg_names_to_ids[leg->getName()]};

            robcogen_end_effector_position = end_effector_position[leg];
            inverse_kinematics_->getJointPosition(robcogen_end_effector_position, iit::dog::LegID(glue_leg_names_to_ids[leg->getName()]), robcogen_joint_position, false, q_violation);
            out[leg->getName()] = std::vector<double>(robcogen_joint_position.data(), robcogen_joint_position.data() + robcogen_joint_position.size());
        }
        
        return out;
    }

    Eigen::Matrix<double, 3, 1> Aliengo::getWholeBodyCOM()
    {        
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

    Eigen::Vector3d Aliengo::getWholeBodyCOM(const robotlib::JointState &joint_state) const
    {

        Eigen::Matrix<double, NJOINTS_TOT, 1> joint_state_matrix = Eigen::Matrix<double, NJOINTS_TOT, 1>::Zero();

        // for (auto leg : legs_)
        // {
        //     for (auto joint : *leg->getJoints())
        //     {
        //         const int joint_id{glue_joint_names_to_ids[joint->getName()]};
        //         joint_state_matrix[joint_id] = joint_state[joint];
        //     }
        // }
        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            joint_state_matrix[joint_id] = joint_state[joint];
        }
        // First updates the coordinate transforms that will be used by the routine
        
        // homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly(joint_state_matrix);
        // homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly(joint_state_matrix);
        // homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly(joint_state_matrix);
        // homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly(joint_state_matrix);
        // homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg(joint_state_matrix);
        // homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg(joint_state_matrix);
        // homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg(joint_state_matrix);
        // homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg(joint_state_matrix);
        // homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg(joint_state_matrix);
        // homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg(joint_state_matrix);
        // homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg(joint_state_matrix);
        // homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg(joint_state_matrix);

        // The actual calculus
        // return getWholeBodyCOM();
        return inertias_->getWholeBodyCOM(joint_state_matrix);
    }

    Eigen::Vector3d Aliengo::getCoMFromBase(const robotlib::JointState &q,
                                            const Eigen::Vector3d &base_orient,
                                            const Eigen::Vector3d &base_pos)
    {
        Eigen::Matrix3d R = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = getWholeBodyCOM(q);
        return base_pos + R.transpose() * offCoM;         //CoM is in the world frame off CoM is in base frame
    }

    Eigen::Vector3d Aliengo::getBaseFromCoM(const robotlib::JointState &q,
                                            const Eigen::Vector3d &base_orient,
                                            const Eigen::Vector3d &CoM)
    {
        Eigen::Matrix3d b_R_w = iit::commons::rpyToRot(base_orient);
        Eigen::Vector3d offCoM = getWholeBodyCOM(q);
        return CoM - b_R_w.transpose() * offCoM;          //CoM is in the world frame off CoM is in base frame
    }

    //compute spatial velocity of the CoM (in base frame) (only joint influence)
    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVel(const robotlib::JointState &q,
                                                            const robotlib::JointState &qd)
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
                                                              const Eigen::Matrix3d &rotationMx,
                                                              const robotlib::JointState &q,
                                                              const robotlib::JointState &qd)
    {
        qd.size();   // TODO: Not used. Created to remove warning

        Eigen::Matrix<double, 6, 1> CoMVel = Eigen::Matrix<double, 6, 1>::Zero();
        //compute joint influence
        //TODO1
        //CoMVel = motionVectorTransform(Eigen::Vector3d::Zero(), rotationMx) * getWholeBodyCOMJacobian(q) * qd;
        //add base motion shifted to the COM

        std::cout << "TODO: getWholeBodyCOMVelFB - compute joint influence\n";

        CoMVel += iit::motionVectorTransform(getWholeBodyCOM(q), rotationMx) * baseVel;

        return CoMVel;
    }

    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                              const Eigen::Matrix3d &rotationMx,
                                                              const robotlib::JointState &q)
    {
        Eigen::Matrix<double, 6, 1> CoMVel = Eigen::Matrix<double, 6, 1>::Zero();

        CoMVel = iit::motionVectorTransform(getWholeBodyCOM(q), rotationMx) * baseVel;

        return CoMVel;
    }

    Eigen::Matrix<double, 6, 1> Aliengo::getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                              const Eigen::Matrix3d &rotationMx,
                                                              const Eigen::Vector3d offset_com)
    {
        Eigen::Matrix<double, 6, 1> CoMVel = Eigen::Matrix<double, 6, 1>::Zero();

        CoMVel = iit::motionVectorTransform(offset_com, rotationMx) * baseVel;

        return CoMVel;
    }

    Eigen::Vector3d Aliengo::getLegContribution(const robotlib::JointState &q) const
    {
        Eigen::Matrix<double, NJOINTS_TOT, 1> joint_state_matrix = Eigen::Matrix<double, NJOINTS_TOT, 1>::Zero();

        for(auto joint : auxiliar_joints_variable_)
        {
            const int joint_id{glue_joint_names_to_ids[joint->getName()]};
            joint_state_matrix[joint_id] = q[joint];
        }

        // First updates the coordinate transforms that will be used by the routine
        homogeneous_transforms_->fr_trunk_X_fr_LF_hipassembly(joint_state_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RF_hipassembly(joint_state_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_LH_hipassembly(joint_state_matrix);
        homogeneous_transforms_->fr_trunk_X_fr_RH_hipassembly(joint_state_matrix);
        homogeneous_transforms_->fr_LF_hipassembly_X_fr_LF_upperleg(joint_state_matrix);
        homogeneous_transforms_->fr_LF_upperleg_X_fr_LF_lowerleg(joint_state_matrix);
        homogeneous_transforms_->fr_RF_hipassembly_X_fr_RF_upperleg(joint_state_matrix);
        homogeneous_transforms_->fr_RF_upperleg_X_fr_RF_lowerleg(joint_state_matrix);
        homogeneous_transforms_->fr_LH_hipassembly_X_fr_LH_upperleg(joint_state_matrix);
        homogeneous_transforms_->fr_LH_upperleg_X_fr_LH_lowerleg(joint_state_matrix);
        homogeneous_transforms_->fr_RH_hipassembly_X_fr_RH_upperleg(joint_state_matrix);
        homogeneous_transforms_->fr_RH_upperleg_X_fr_RH_lowerleg(joint_state_matrix);

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

    double Aliengo::getTrunkMass() const
    {
        return inertias_->getTrunkMass();
    }

    double Aliengo::getLegsMass() const
    {
        return inertias_->getLegMass();
    }

    void Aliengo::setInvKinTimePeriod(const double& period) const
    {
        inverse_kinematics_->setTimePeriod(period);
    } 

    void Aliengo::setTrunkCom(const Eigen::Vector3d &trunk_com) const
    {
        robot_params_->setValue_trunk_com_x(trunk_com(0));
        robot_params_->setValue_trunk_com_y(trunk_com(1));
        robot_params_->setValue_trunk_com_z(trunk_com(2));
    }

    void Aliengo::setTrunkMass(const double& trunk_mass) const
    {
        robot_params_->setValue_trunk_mass(trunk_mass);   
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
        std::string urdf_path("/usr/include/robots/aliengo.urdf");

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
