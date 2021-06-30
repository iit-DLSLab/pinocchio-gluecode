#ifndef IIT_ROBOGEN__ALIENGO_TRAITS_H_
#define IIT_ROBOGEN__ALIENGO_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace Aliengo {

struct Traits {
    typedef typename Aliengo::JointState JointState;

    typedef typename Aliengo::JointIdentifiers JointID;
    typedef typename Aliengo::LinkIdentifiers  LinkID;

    typedef typename Aliengo::HomogeneousTransforms HomogeneousTransforms;
    typedef typename Aliengo::MotionTransforms MotionTransforms;
    typedef typename Aliengo::ForceTransforms ForceTransforms;

    typedef typename Aliengo::dyn::InertiaProperties InertiaProperties;
    typedef typename Aliengo::dyn::ForwardDynamics FwdDynEngine;
    typedef typename Aliengo::dyn::InverseDynamics InvDynEngine;
    typedef typename Aliengo::dyn::JSIM JSIM;

    static const int joints_count = Aliengo::jointsCount;
    static const int links_count  = Aliengo::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return Aliengo::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return Aliengo::orderedLinkIDs;
}

}
}

#endif
