/*
 * params.h
 *
 *  Created on: Sep 10, 2014
 *      Author: marco
 */

#ifndef IIT_ROBOTS_ALIENGO_HYDRAULICS_PARAMS_H_
#define IIT_ROBOTS_ALIENGO_HYDRAULICS_PARAMS_H_

#include <iit/commons/hydraulics.h>


namespace iit {
namespace Aliengo {


struct HydraulicsParameters {
        iit::hydr::PipelineProperties pipeline;
        iit::hydr::MotorProperties    HAA_motor;
        iit::hydr::MotorProperties    HFE_motor;
        iit::hydr::CylinderProperties KFE_cylinder;
        iit::hydr::ValveProperties    valveParams;

        HydraulicsParameters();
};

}
}


#endif
