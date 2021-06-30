/*
 * footest.cpp
 *
 *  Created on: May 21, 2013
 *      Author: mfrigerio
 */

#include <iit/robots/aliengo/declarations.h>
#include <iit/robots/aliengo/leg_data_map.h>
#include <iit/robots/aliengo/joints_pid_control.h>

using namespace iit::Aliengo;

static JointsPIDControl PIDCtrl(0.0, 0.0);
static JointsPIDgains PIDGains(1.0);
static JointsPIDControl::References Ref(1.0);
static JointsPIDControl::States Measured(0.0);
static JointDataMap<double> commands(0.0);
static JointsPIDerrors Errors;

static void compute_pid()
{

}

int main() {

    double Ts = 0.001; //Sampling time;
    double Tf = 15 * Ts;
    PIDCtrl = JointsPIDControl(Ts, Tf);
    //set gains
    iit::Aliengo::JointIdentifiers jid;
    for(int i=0; i<jointsCount; i++) {
        jid = orderedJointIDs[i];
        PIDGains[jid].p = 0.0;
        PIDGains[jid].i = 1000.0;
        PIDGains[jid].d =0.0;
        Ref[jid]= 1.0;
        Measured[jid]= 0.0;

    }
    //do at the beginning
    PIDCtrl.setGains(PIDGains);
    //JointsPIDgains gains = PIDCtrl.getGains();
    //std::cout<< gains[jid].i << std::endl;

    //for any loop
    for (int time=0; time<100; time++){
        //switch error sign to see antiwindup works
        if (time > 50)
            Ref = -1;

        //compute pid
        //PIDCtrl.pid(Ref, Measured);
        PIDCtrl.pidAntiWindup(Ref, Measured , 10, -10, commands, 1);
        Errors = PIDCtrl.getErrors();
        const JointsPIDterms& pid_terms = PIDCtrl.getTerms();

        for(int i=0; i<jointsCount; i++) {
            jid = orderedJointIDs[i];
            commands[jid] = pid_terms[jid].sumTerms();

        }
        std::cout <<"time ="<<time<<" ref " <<Ref[jid]<< " error "<<    Errors[jid].current<<" command "<<commands[jid]<<std::endl;

    }
    //comopute_pid(); // you can redirect stdout to get a csv log file
	return 0;
}
