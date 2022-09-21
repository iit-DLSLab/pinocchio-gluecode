/*
 * hydraulics.cpp
 *
 *  Created on: May 20, 2013
 *      Author: mfrigerio
 */

#include <iit/commons/hydraulics.h>
#include <iit/commons/control.h>

#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/joint_data_map.h>
#include <iit/commons/four_bar_linkage.h>

#include <iit/robots/aliengo/hydraulics/dynamics.h>
#include <iit/robots/aliengo/joint_utils.h>
#include <iit/robots/aliengo/mechanical_constants.h>

using namespace iit;

using namespace iit::commons;

using namespace iit::dog;

Aliengo::HydraulicsDynamics::HydraulicsDynamics(
    const HydraulicsParameters& params,
    const ValvePortToVoltageMap& port2voltage,
    const ForceToValvePortMap&  force2port)
    :
    hydrParams(params),
    dynamicsTerms(0.0),
    linearizationTerms(0.0),
    valvePortToVoltage(port2voltage),
    forceToValvePort(force2port)
{

}



void Aliengo::HydraulicsDynamics::estimateMotorChamberPressures(
        JointIdentifiers j,
        double tau_force,
        iit::hydr::Pressures& pressures) const
{
    iit::hydr::motorChamberPressures(
            isHAA(j) ? hydrParams.HAA_motor : hydrParams.HFE_motor,
            forceToValvePort[j].getPositiveForcePort(),
            forceToValvePort[j].getNegativeForcePort(),
            tau_force, pressures);
}


void Aliengo::HydraulicsDynamics::computeDynamicsTerms(
        const dog::JointState& q,
        const dog::JointState& qd,
        const Pressures& pressures)
{
    dog::JointIdentifiers j;
    for (int i = 0; i<dog::jointsCount; i++)
    {
        j = dog::orderedJointIDs[i];
        if( Aliengo::hasRotaryActuator(j) )
        {
            motor_dynamics_terms(j, q(j), qd(j), pressures[j]);
        } else {
            cylinder_dynamics_terms(j, q(j), qd(j), pressures[j]);
        }
    }
}


void Aliengo::HydraulicsDynamics::computeLinearizationTerms(const LinearizationGains& gains)
{
    dog::JointIdentifiers j;
    for (int i = 0; i<dog::jointsCount; i++)
    {
        j = dog::orderedJointIDs[i];
        iit::hydr::computeLinearizationTerms(dynamicsTerms[j], gains[j], linearizationTerms[j]);
    }
}

void Aliengo::HydraulicsDynamics::applyLinearizationTerms(
        const dog::JointDataMap<double>& input_commands,
        dog::JointDataMap<double>& output_commands) const
{
    dog::JointIdentifiers j;
    for (int i = 0; i<dog::jointsCount; i++)
    {
        j = dog::orderedJointIDs[i];
        output_commands[j] = iit::hydr::applyLinearizationTerms(linearizationTerms[j], input_commands[j]);
    }
}



void Aliengo::HydraulicsDynamics::motor_dynamics_terms(
        const dog::JointIdentifiers& j,
        const double& q, const double& qd,
        const iit::hydr::Pressures& pressures)
{
    iit::hydr::ActuatorState motorState;

    motorState.q  = jointStateToMotorState(j,q);
    motorState.qd = qd;
    motorState.activePort = activeValvePorts[j];
    debugActuator[j].qd =  motorState.qd;
    debugActuator[j].q =  motorState.q;

    iit::hydr::feedback_linearization_motor(
            hydrParams.pipeline,
            isHAA(j) ? hydrParams.HAA_motor : hydrParams.HFE_motor,
            hydrParams.valveParams,
            motorState,
            pressures,
            forceToValvePort[j].getPositiveForcePort(),
            valvePortToVoltage[j],
            dynamicsTerms[j].f, dynamicsTerms[j].g);
}


void Aliengo::HydraulicsDynamics::cylinder_dynamics_terms(
        const dog::JointIdentifiers& j,
        const double& q, const double& qd,
        const iit::hydr::Pressures& pressures)
{
    iit::hydr::ActuatorState cylinderState;
    iit::commons::FBLConsts fblConsts;
    fblConsts.akh = fbl::akh;
    fblConsts.cj =  fbl::cj;
    fblConsts.jk = fbl::jk;
    fblConsts.ak = fbl::ak;
    fblConsts.bk = fbl::bk;
    fblConsts.ar = fbl::ar;
    fblConsts.br = fbl::br;
    fblConsts.angle_KB_lleg_xaxis = fbl::angle_KB_lleg_xaxis;
    fblConsts.cyl_retracted = fbl::cyl_retracted;
    FBLStatus fbl(fblConsts);
    fbl.computeNumericJacobian(q);

    //actuator state are piston position and velocity
    cylinderState.q  = jointStateToCylinderState(j, fbl);
    cylinderState.qd =  qd * fbl.jac_num; //piston velocity is the joint velocity multiplied by lever arm
    cylinderState.activePort = activeValvePorts[j];
    debugActuator[j].qd =  cylinderState.qd;
    debugActuator[j].q =  cylinderState.q;


    iit::hydr::feedback_linearization_cylinder(
            hydrParams.pipeline,
            hydrParams.KFE_cylinder,
            hydrParams.valveParams,
            cylinderState,
            pressures,
            forceToValvePort[j].getPositiveForcePort(),
            valvePortToVoltage[j],
            dynamicsTerms[j].f, dynamicsTerms[j].g);
}

