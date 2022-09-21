#ifndef IIT_ALIENGO_SHIN_JACOBIANS_H_
#define IIT_ALIENGO_SHIN_JACOBIANS_H_

#include <iit/commons/dog/shin_jacobians.h>
#include <iit/commons/dog/leg_data_map.h>

#include <iit/commons/dog/declarations.h>
#include <iit/robots/aliengo/jacobians.h>
#include <iit/robots/aliengo/feet_jacobians.h>

#include <iit/rbd/utils.h>

namespace iit {
namespace Aliengo {



class ShinJacobians : public dog::ShinJacobians
{
typedef dog::JointState JointState;


public:
    ShinJacobians(const dog::KinDynParams & param_getter) :
        param_getter_(param_getter), jacobians_(param_getter_),
        feet_jacobians_(jacobians_)
    {
    }
    ~ShinJacobians() {}

    dog::FootJac getShinJacobian(const JointState& q,
                                  const double& contact_point,
                                  const dog::LegID& leg) {


        // TODO
        // the default param getter has no shins for now,
        // returning the foot jacobians always
        //
        /*switch(leg){
        case dog::LF:
            param_getter_setValue_LF_shin(contact_point);
            return jacobians_.fr_trunk_J_LF_shin(q);
        case dog::RF:
            param_getter_setValue_RF_shin(contact_point);
            return jacobians_.fr_trunk_J_RF_shin(q);
        case dog::LH:
            param_getter_setValue_LH_shin(contact_point);
            return jacobians_.fr_trunk_J_LH_shin(q);
        case dog::RH:
            param_getter_setValue_RH_shin(contact_point);
            return jacobians_.fr_trunk_J_RH_shin(q);
        }
        return dog::FootJac::Identity();*/
        return feet_jacobians_.getFootJacobian(q,leg);
    }

private:
    const dog::KinDynParams& param_getter_;
    Aliengo::Jacobians jacobians_;
    Aliengo::FeetJacobians feet_jacobians_;
};


}
}


#endif
