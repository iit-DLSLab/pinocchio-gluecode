/*
 * hydraulics.h
 *
 *  Created on: Apr 29, 2013
 *      Author: phd
 */

#ifndef _IIT_ALIENGO_COMMONS_HYDRAULICS_H_
#define _IIT_ALIENGO_COMMONS_HYDRAULICS_H_

#include <iit/commons/hydraulics.h>

#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/joint_data_map.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/dog/joint_id_tricks.h>
#include <iit/commons/four_bar_linkage.h>


#include "params.h"
#include "../internals.h"


//#include "valves_config.h"

namespace iit {
namespace Aliengo {

typedef iit::hydr::ValvePort Valve;


template<class T>
using JointDataMap = iit::dog::JointDataMap<T>;

typedef iit::dog::JointIdentifiers JointIdentifiers;
typedef iit::dog::JointState JointState;

typedef JointDataMap<iit::hydr::Pressures>          Pressures;
typedef JointDataMap<iit::hydr::HydDynTerms>        DynamicsTerms;
typedef JointDataMap<iit::hydr::LinearizationTerms> LinearizationTerms;
typedef JointDataMap<iit::hydr::LinearizationGains> LinearizationGains;
typedef JointDataMap<Valve::Port>                   ValvePorts;
typedef JointDataMap<iit::hydr::PortToVoltageSign>  ValvePortToVoltageMap;
typedef JointDataMap<iit::hydr::ForceSignToPort>    ForceToValvePortMap;

class HydraulicsDynamics
{

public:
    HydraulicsDynamics();

    HydraulicsDynamics(
        const HydraulicsParameters&,
        const ValvePortToVoltageMap&,
        const ForceToValvePortMap&);

    void setHydraulicsParameters(const HydraulicsParameters&);
    void configure(const ValvePortToVoltageMap&, const ForceToValvePortMap&);

    void setActiveValvePort(JointIdentifiers, Valve::Port);

    void estimateCylinderChamberPressures(
            JointIdentifiers j,
            double force,
            iit::hydr::Pressures& press) const;

    void estimateMotorChamberPressures(
            JointIdentifiers j,
            double tau_force,
            iit::hydr::Pressures& press) const;

    void computeDynamicsTerms(
            const JointState& q,
            const JointState& qd,
            const Pressures& pressures);

    void computeLinearizationTerms(
            const LinearizationGains& linGains);

    void applyLinearizationTerms(
            const JointDataMap<double>& force_ctrl_commands,
            JointDataMap<double>& output_commands) const;

    double jointStateToCylinderState(JointIdentifiers, const iit::commons::FBLStatus&) const;
    double jointStateToMotorState(JointIdentifiers, const double&) const;

    const DynamicsTerms& getDynamicsTerms() const;
    const LinearizationTerms& getLinearizationTerms() const;

    iit::Aliengo::JointDataMap<iit::hydr::ActuatorState> debugActuator;


private:
    void motor_dynamics_terms(
            const JointIdentifiers& haa,
            const double& q, const double& qd,
            const iit::hydr::Pressures& pressures);

    void cylinder_dynamics_terms(
            const JointIdentifiers& sagittalj,
            const double& q, const double& qd,
            const iit::hydr::Pressures& pressures);
private:
    static const JointIdentifiers haa_joints_id[4];
    HydraulicsParameters hydrParams;
    DynamicsTerms        dynamicsTerms;
    LinearizationTerms   linearizationTerms;
    ValvePorts           activeValvePorts;
    ValvePortToVoltageMap valvePortToVoltage;
    ForceToValvePortMap  forceToValvePort;
};



inline HydraulicsDynamics::HydraulicsDynamics() :
        hydrParams(), dynamicsTerms(0.0), linearizationTerms(0.0)
{}

inline void HydraulicsDynamics::setHydraulicsParameters(
        const HydraulicsParameters& params)
{
    hydrParams = params;
}

inline void HydraulicsDynamics::configure(
        const ValvePortToVoltageMap& port2Volt, const ForceToValvePortMap& force2Port)
{
    valvePortToVoltage = port2Volt;
    forceToValvePort   = force2Port;
}


inline void HydraulicsDynamics::setActiveValvePort(
        JointIdentifiers j, Valve::Port p)
{
    activeValvePorts[j] = p;
}


inline void HydraulicsDynamics::estimateCylinderChamberPressures(
        JointIdentifiers j,
        double force,
        iit::hydr::Pressures& pressures) const
{
    iit::hydr::cylinderChamberPressures(
            hydrParams.KFE_cylinder,
            forceToValvePort[j].getPositiveForcePort(),
            forceToValvePort[j].getNegativeForcePort(),
            force, pressures);
}


inline double HydraulicsDynamics::jointStateToCylinderState(
        JointIdentifiers j, const iit::commons::FBLStatus& fbl) const
{
    //double extension = fbl.cylinder - fbl::cyl_retracted;
    return
    (forceToValvePort[j].getPositiveForcePort() == iit::hydr::ValvePort::A) ?
            fbl.extension  :  hydrParams.KFE_cylinder.stroke - fbl.extension;
}

inline double HydraulicsDynamics::jointStateToMotorState(JointIdentifiers j, const double& q) const
{
    return
    isHAA(j) ? q - internal::misc_cfg.HAAJointToMotorOffset[ toLegID(j) ]
             : q - internal::misc_cfg.HFEJointToMotorOffset[ toLegID(j) ];
}


inline const DynamicsTerms& HydraulicsDynamics::getDynamicsTerms() const {
    return dynamicsTerms;
}
inline const LinearizationTerms& HydraulicsDynamics::getLinearizationTerms() const {
    return linearizationTerms;
}




}
}





#endif
