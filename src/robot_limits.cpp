#include "iit/robots/aliengo/robot_limits.h"
#include "iit/robots/aliengo/mechanical_constants.h"
#include <iostream>

namespace iit {
namespace Aliengo {

Limits::Limits()
{
    max_actuator_effort_ << 173.0, 208.0, 7142.0,  173.0, 208.0, 7142.0,
                            173.0, 208.0, 7142.0,  173.0, 208.0, 7142.0; //N //TODO change
    fblConsts.akh = fbl::akh;
    fblConsts.cj =  fbl::cj;
    fblConsts.jk = fbl::jk;
    fblConsts.ak = fbl::ak;
    fblConsts.bk = fbl::bk;
    fblConsts.ar = fbl::ar;
    fblConsts.br = fbl::br;
    fblConsts.angle_KB_lleg_xaxis = fbl::angle_KB_lleg_xaxis;
    fblConsts.cyl_retracted = fbl::cyl_retracted;

    fblSts.reset(new iit::commons::FBLStatus(fblConsts));
}

Limits::~Limits(){}

void Limits::setMaxEffort(const dog::JointState & max_effort)
{
    max_actuator_effort_ = max_effort;
}

dog::JointState Limits::getTorqueLimits(const iit::dog::JointState & q)
{

    dog::JointState tau_max;
    tau_max.setZero();

    iit::dog::JointIdentifiers j_id;

    for (int i = 0; i<iit::dog::jointsCount; i++){
        j_id = iit::dog::JointIdentifiers(i);

        if ((j_id==iit::dog::LF_KFE)||(j_id==iit::dog::RF_KFE)||(j_id==iit::dog::LH_KFE)||(j_id==iit::dog::RH_KFE))  {
            fblSts->computeNumericJacobian(q(j_id));
            tau_max(j_id) = max_actuator_effort_(j_id) * fblSts->jac_num;
            //to check the values are correct
            //double lever_arm = 0.00006412*pow(q(j_id),6) + 0.0002046*pow(q(j_id),5) -0.0006317*pow(q(j_id),4)   -0.0007159*pow(q(j_id),3) +0.00008877*pow(q(j_id),2) -0.01217*q(j_id)  +  0.01663;
            //std::cout<<"Joint id   "<<j_id<<std::endl;
            //std::cout<<"Numeric: "<< fblSts->jac_num<<" Poly: " <<lever_arm<<std::endl;
        }else{
            tau_max(j_id) = max_actuator_effort_(j_id);}
    }
    return tau_max;
}


}
}

