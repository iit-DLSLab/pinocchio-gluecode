/*
 * internals.h
 *
 *  Created on: Sep 10, 2014
 *      Author: marco
 */

#ifndef IIT_ROBOTS_ALIENGO_INTERNALS_H_
#define IIT_ROBOTS_ALIENGO_INTERNALS_H_

#include <iit/commons/dog/leg_data_map.h>

namespace iit {
namespace Aliengo {

namespace internal {

struct MagicNumbers {
    MagicNumbers();

    dog::LegDataMap<double> HFEJointToMotorOffset;
    dog::LegDataMap<double> HAAJointToMotorOffset;

};

extern const MagicNumbers misc_cfg;

}
}
}



#endif
