#ifndef IIT_ALIENGO_LIMITS_H
#define IIT_ALIENGO_LIMITS_H

#include <iit/commons/dog/robot_limits.h>
#include <iit/commons/four_bar_linkage.h>
#include <memory>

namespace iit
{

namespace Aliengo {

class Limits : public dog::LimitsBase
{
public:
    Limits();
    virtual ~Limits();

    void setMaxEffort(const dog::JointState & max_effort);

    dog::JointState getTorqueLimits(const dog::JointState & q);
private:
   dog::JointState  max_actuator_effort_;
   iit::commons::FBLConsts fblConsts;
   std::shared_ptr<iit::commons::FBLStatus> fblSts;
};


}
}


#endif
