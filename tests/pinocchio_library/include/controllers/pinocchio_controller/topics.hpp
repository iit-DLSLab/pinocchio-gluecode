#ifndef PINOCCHIO_CONTROLLER_TOPICS_HPP
#define PINOCCHIO_CONTROLLER_TOPICS_HPP

#include "dls2/topics/utils.hpp"
#include <dls_messages/dds/control_signalPubSubTypes.hpp> // # off-the-shelf message

#include "dls_messages/dds/debug_pinocchioPubSubTypes.hpp"
namespace dls
{
    namespace topics
    {
        namespace pinocchio_controller{
            dls::topicType tau = dls::topicType("pinocchio_controller", new ControlSignalMsgPubSubType());
            // define dynamic topic for debug message
            // -- read xml message profile
            #ifndef DEBUG_MSG_PATH
            #define DEBUG_MSG_PATH "path_to_debug_msg_path"
            #endif
            dls::topicType debug = dls::topicType("pinocchio_controller_debug", new DebugPinocchioMsgPubSubType());
        }
    }
}
#endif /* end of include guard: PINOCCHIO_CONTROLLER_TOPICS_HPP */