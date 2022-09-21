
#include <iit/robots/aliengo/internals.h>
#include <iit/robots/aliengo/mechanical_constants.h>
#include <iit/commons/dog/leg_data_map.h>

using namespace iit::dog;

const iit::Aliengo::internal::MagicNumbers iit::Aliengo::internal::misc_cfg;

iit::Aliengo::internal::MagicNumbers::MagicNumbers()
{
    HFEJointToMotorOffset[LF] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[RF] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[LH] =  LF_HFE_JOINT_MOTOR_OFF;
    HFEJointToMotorOffset[RH] =  LF_HFE_JOINT_MOTOR_OFF;

    HAAJointToMotorOffset[LF] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[RF] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[LH] = LF_HAA_JOINT_MOTOR_OFF;
    HAAJointToMotorOffset[RH] = LF_HAA_JOINT_MOTOR_OFF;

}
