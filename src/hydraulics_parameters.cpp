/*
 * hydraulics_parameters.cpp
 *
 *  Created on: Sep 10, 2014
 *      Author: marco
 */

#include <iit/robots/aliengo/hydraulics/params.h>


iit::Aliengo::HydraulicsParameters::HydraulicsParameters()
{
    pipeline.oilBulkMod = 1.8181E9;
    pipeline.volume     = 8.17E-6; // tube 0.65 long, internal diameter 0.004 (everything in meters)

    // TODO check the following
    HAA_motor.volumetricDisp = 6.87E-6;
    HAA_motor.totVolume      = 12.0E-6; //range of motion (100°) times vol displacement
    HAA_motor.unusedVolume[iit::hydr::ValvePort::A] = 0.0; // TODO need to update 
    HAA_motor.unusedVolume[iit::hydr::ValvePort::B] = 0.0; // TODO need to update 

    // Rotary actuator code HPHP1045
    HFE_motor.volumetricDisp = 1.432E-5; // [m³/rad]
    HFE_motor.totVolume      = 6.93E-5;  // [m³]
    HFE_motor.unusedVolume[iit::hydr::ValvePort::A] = 0.85E-6; // TODO need to update 
    HFE_motor.unusedVolume[iit::hydr::ValvePort::B] = 0.85E-6; // TODO need to update 


    KFE_cylinder.pistonArea[hydr::ValvePort::A] = 3.14E-4; // [m²]
    KFE_cylinder.pistonArea[hydr::ValvePort::B] = 2.00E-4; // [m²]
    KFE_cylinder.stroke = 0.07; // [m]

    valveParams.gain = 6.12E-8; // nominal flow (12 lt/min), nominal pressure drop (3000psi), nominal input of the valve (1=100%)
    valveParams.gain_f =  valveParams.gain;
}
