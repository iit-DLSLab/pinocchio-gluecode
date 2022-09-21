/*
 * joint_ids_tricks.h
 *
 *  Created on: Nov 17, 2014
 *      Author: mfrigerio
 */

#ifndef IIT_ROBOTS_ALIENGO_JOINT_IDS_TRICKS_H_
#define IIT_ROBOTS_ALIENGO_JOINT_IDS_TRICKS_H_

#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/leg_data_map.h>

namespace iit
{
namespace Aliengo
{

inline bool hasRotaryActuator(dog::JointIdentifiers j)
{
    return ! (j==dog::LF_KFE || j==dog::RF_KFE || j==dog::LH_KFE || j==dog::RH_KFE);
}

}
}

#endif
