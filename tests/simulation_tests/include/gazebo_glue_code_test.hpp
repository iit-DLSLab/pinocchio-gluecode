#ifndef GAZEBO_GLUECODE_TEST_H
#define GAZEBO_GLUECODE_TEST_H

// Gazebo includes
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ContactSensor.hh>

#include <ignition/transport.hh>

#include "dls2/util/messaging/dds_participant.hpp"
#include "dls2/log/log.hpp"
#include "dls2/msg_wrappers/t265_odometry.hpp"
#include "dls2/msg_wrappers/signal_writer.hpp"
#include "dls2/util/time/time.hpp"

#include "robotlib/robot_factory.hpp"
#include "robotlib/utils/utils.hpp"
#include "dls2/math/algebra.hpp"
#include "dls2/math/rotations.hpp"

#include "dls_messages/dds/gazebo_glue_code_test.h"

#include <chrono>

namespace dls
{

   /**
    * @class This class tests some of the Robotlib functions by accessing to ground truth data from Gazebo.
    */
   class GazeboPluginGlueTest : public gazebo::ModelPlugin
   {
   public:

        /**
        * @brief Constructor
        */
        GazeboPluginGlueTest();

         /*!
        * @brief Load the plugin.
        * @param[in] model Pointer to the Model.
        * @param[in] element SDF element for the plugin.
        */
        void Load
        (
            gazebo::physics::ModelPtr,
            sdf::ElementPtr
        ) override;

        /**
        * @brief Reads blind state from Gazebo
        */
        void run();
        
    private:
        gazebo::physics::ModelPtr sim_model;
        gazebo::event::ConnectionPtr update_connection;
        std::vector<gazebo::physics::JointPtr> sim_joints;

        std::shared_ptr<robotlib::RobotBase> pRobot;	///< A pointer to the robot model
        
        std::shared_ptr<dls::DDSParticipant> ddslink;

        Time time_factor;
        GazeboGlueCodeTestMsg msg;
    };

   GZ_REGISTER_MODEL_PLUGIN(GazeboPluginGlueTest);
}

#endif  // GAZEBO_PINOCCHIO_TEST_H