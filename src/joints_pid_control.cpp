/*
 * PIDControl.cpp
 *
 *  Created on: May 22, 2013
 *      Author: mfrigerio
 */

#include "iit/robots/aliengo/joints_pid_control.h"
#include <iit/commons/dog/declarations.h>

using namespace iit;
using namespace iit::Aliengo;
using namespace iit::dog;


void JointsPIDControl::pid(
        const References& ref,
        const States&     actual)
{
    static JointIdentifiers jid;

    for(int i=0; i<jointsCount; i++)
    {
        jid = orderedJointIDs[i];
        singleJointPID(jid, ref[jid], actual[jid]);
    }
}
