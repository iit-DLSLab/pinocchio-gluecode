#ifndef PINOCCHIO_CONTROLLER_HPP
#define PINOCCHIO_CONTROLLER_HPP

// periodic plugin header
#include <dls2/plugin/periodic_app_plugin.hpp>
// wrappers
#include <dls2/msg_wrappers/blind_state.hpp> //  off-the-shelf wrapper
#include <dls2/msg_wrappers/base_state.hpp> //  off-the-shelf wrapper
#include <dls2/msg_wrappers/control_signal.hpp> //  off-the-shelf wrapper
#include <dls2/msg_wrappers/trajectory_generator.hpp> //  off-the-shelf wrapper
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include "dls_messages/dds/debug_pinocchio.h"
namespace controllers
{
    class PinocchioController : public dls::PeriodicAppPlugin
    {
    public:
        PinocchioController (
            const std::string &ID
            , const std::shared_ptr<robotlib::RobotBase> robot);

        ~PinocchioController();

        void run(const std::chrono::system_clock::time_point &time) override;

        std::string where() override;

        AppStatus eStop() override { return getStatus(); }
        
        virtual bool deactivation(const std::chrono::system_clock::time_point& time) override;

        void runController();

        Eigen::VectorXd inverseKinematics(const std::string& ee_name, const Eigen::VectorXd& q_guess, const Eigen::VectorXd& q_gt, const Eigen::Matrix4d& pose_des);

        Eigen::VectorXd inverseKinematicsFrame(const std::string& ee_name, const Eigen::VectorXd& q_guess, const Eigen::VectorXd& q_gt, const Eigen::Matrix4d& pose_des);

        bool setQIncrement();
        // eprosima::fastrtps::types::DynamicData_ptr debug_msg;
        bool setPoseIncrement();

    private:
        std::shared_ptr<robotlib::RobotBase> pRobot;
        //! Input signals
		BaseState input_base_state;
		BlindState input_blind_state;

		//! Output signals
		ControlSignal output_tau;
        TrajectoryGenerator output_traj_gen;

        pinocchio::Model robot_model;
        pinocchio::Data robot_data;

        DebugPinocchioMsg debug_msg;

        Eigen::VectorXd q_increment;
        Eigen::Vector<double,6> pose_increment;


        std::map<int,int> idx_map = {{3,6},{4,7},{5,8}};

        Eigen::VectorXd reorderJoints(const Eigen::VectorXd& data) const;

        // std::ofstream outFile;
    };
} // namespace controllers

#endif // end of include guard: PINOCCHIO_CONTROLLER_HPP
