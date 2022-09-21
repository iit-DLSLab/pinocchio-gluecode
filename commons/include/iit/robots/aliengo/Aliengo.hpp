#ifndef ALIENGO_HPP_EN0TYZCZ
#define ALIENGO_HPP_EN0TYZCZ

#include "iit/commons/dog/dog.hpp"

// #include "declarations.h"
#include "default_parameters_getter.h"
#include "dynamics_parameters.h"
#include "feet_contact_forces.h"
#include "feet_jacobians.h"
#include "forward_dynamics.h"
#include "forward_kinematics.h"
#include "Aliengo.hpp"
#include "inertia_properties.h"
#include "internals.h"
#include "inverse_dynamics.h"
#include "inverse_kinematics.h"
#include "jacobians.h"
#include "joint_utils.h"
#include "joints_pid_control.h"
#include "jsim.h"
#include "kinematics_parameters.h"
#include "mechanical_constants.h"
#include "robot_limits.h"
#include "RobotLengths.h"
#include "shin_jacobians.h"
#include "traits.h"
#include "transforms.h"


class Aliengo : public iit::dog::Dog
{
	Aliengo::Aliengo():
		Dog
		(
			std::make_shared<iit::Aliengo::dyn::JSimBase>               (),
			std::make_shared<iit::dog::TestFK>                          (),
			std::make_shared<iit::dog::JointBoolMap>                    (),
			std::make_shared<iit::Aliengo::RobotLenghts>                (),
			std::make_shared<iit::Aliengo::ForwardKinematics>           (),
			std::make_shared<iit::Aliengo::dog::TestK>                  (),
			std::make_shared<iit::Aliengo::FeetContactForces>           (),
			std::make_shared<iit::Aliengo::InverseKinematics>           (),
			std::make_shared<iit::Aliengo::dyn::InverseDynamicsBase>    (),
			std::make_shared<iit::Aliengo::HomogeneousTransformsBase>   (),
			std::make_shared<iit::Aliengo::MotionTransformsBase>        (),
			std::make_shared<iit::Aliengo::ForceTransformsBase>         (),
			std::make_shared<iit::Aliengo::DefaultParamsGetter>         (),
			std::make_shared<iit::Aliengo::LimitsBase>                  (),
			std::make_shared<iit::Aliengo::dyn::InertiaPropertiesBase>  (),
			std::make_shared<iit::Aliengo::FeetJacobians>               (),
			std::make_shared<iit::dog::LegBoolMap>                      (),
			std::make_shared<iit::Aliengo::ShinJacobians>               ()
		)
	{}
};


#endif
