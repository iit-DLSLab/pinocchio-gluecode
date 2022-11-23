#include "robcogen/jacobians.h"
#include "dog/leg_data_map.h"
#include "dog/joint_id_tricks.h"

using namespace iit::dog;

iit::Aliengo::Jacobians::Jacobians
    (const dog::KinDynParams& getter_lengths)
     : 
    fr_trunk_J_LF_foot(lengths_values), 
    fr_trunk_J_RF_foot(lengths_values), 
    fr_trunk_J_LH_foot(lengths_values), 
    fr_trunk_J_RH_foot(lengths_values)
    ,
    valuesGetter_lengths(getter_lengths)
{
    updateParameters();
}


void iit::Aliengo::Jacobians::updateParameters() {
    lengths_values.haa_x = valuesGetter_lengths.getValue_haa_x();
    lengths_values.haa_y = valuesGetter_lengths.getValue_haa_y();
    lengths_values.haa_hfe = valuesGetter_lengths.getValue_haa_hfe();
    lengths_values.upper_leg = valuesGetter_lengths.getValue_upper_leg();
    lengths_values.foot_x = valuesGetter_lengths.getValue_foot_x();
}

namespace iit {
namespace Aliengo {
dog::FootJac Jacobians::getFootJacobianXY(const JointState & q,
                                          const iit::dog::LegID& leg,
                                          const double& foot_x,
                                          const double& foot_y)
{
    double sin__q_HAA__ = std::sin( q(iit::dog::toJointID(leg, iit::dog::HAA)));
    double sin__q_HFE__ = std::sin( q(iit::dog::toJointID(leg, iit::dog::HFE)));
    double sin__q_KFE__ = std::sin( q(iit::dog::toJointID(leg, iit::dog::KFE)));
    double cos__q_HAA__ = std::cos( q(iit::dog::toJointID(leg, iit::dog::HAA)));
    double cos__q_HFE__ = std::cos( q(iit::dog::toJointID(leg, iit::dog::HFE)));
    double cos__q_KFE__ = std::cos( q(iit::dog::toJointID(leg, iit::dog::KFE)));

    switch(leg){
        case dog::LF:
            jacobian_(0,0) = 0.0;
            jacobian_(0,1) =((((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__)) - ( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HFE__));
            jacobian_(0,2) = (((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__));
            jacobian_(1,0) = ((((((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  sin__q_HFE__) - (( foot_x *  cos__q_HAA__) *  cos__q_HFE__)) *  cos__q_KFE__)) - (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  sin__q_HAA__));
            jacobian_(1,1) = (((((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  sin__q_HAA__) *  cos__q_HFE__) + (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  sin__q_HFE__));
            jacobian_(1,2) = ((((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  sin__q_HAA__) *  cos__q_HFE__) + (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));
            jacobian_(2,0) = (((((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  cos__q_HAA__));
            jacobian_(2,1) = (((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  sin__q_HFE__));
            jacobian_(2,2) = ((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));

        break;
        case dog::RF:
             jacobian_(0,0) = 0.0;
             jacobian_(0,1) = ((((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__)) - ( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HFE__));
             jacobian_(0,2) = (((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__));
             jacobian_(1,0) = (((((((- foot_y *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  cos__q_HFE__)) + ( valuesGetter_lengths.getValue_haa_hfe() *  sin__q_HAA__));
             jacobian_(1,1) = (((((( foot_y *  sin__q_HAA__) *  sin__q_HFE__) - (( foot_x *  sin__q_HAA__) *  cos__q_HFE__)) *  sin__q_KFE__) + ((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) - (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  sin__q_HFE__));
             jacobian_(1,2) = ((((( foot_y *  sin__q_HAA__) *  sin__q_HFE__) - (( foot_x *  sin__q_HAA__) *  cos__q_HFE__)) *  sin__q_KFE__) + ((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));
             jacobian_(2,0) = (((((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  cos__q_HAA__));
             jacobian_(2,1) = (((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  sin__q_HFE__));
             jacobian_(2,2) = ((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));
                break;

        case dog::LH:
            jacobian_(0,0) = 0.0;
            jacobian_(0,1) = ((((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__)) - ( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HFE__));
            jacobian_(0,2) = (((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__));
            jacobian_(1,0) = ((((((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  sin__q_HFE__) - (( foot_x *  cos__q_HAA__) *  cos__q_HFE__)) *  cos__q_KFE__)) - (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  sin__q_HAA__));
            jacobian_(1,1) = (((((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  sin__q_HAA__) *  cos__q_HFE__) + (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  sin__q_HFE__));
            jacobian_(1,2) = ((((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  sin__q_HAA__) *  cos__q_HFE__) + (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));
            jacobian_(2,0) = (((((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  cos__q_HAA__));
            jacobian_(2,1) = (((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  sin__q_HFE__));
            jacobian_(2,2) = ((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));

        break;

        case dog::RH:
            jacobian_(0,0) = 0.0;
            jacobian_(0,1) = ((((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__)) - ( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HFE__));
            jacobian_(0,2) = (((( foot_y *  cos__q_HFE__) + ( foot_x *  sin__q_HFE__)) *  sin__q_KFE__) + ((( foot_y *  sin__q_HFE__) - ( foot_x *  cos__q_HFE__)) *  cos__q_KFE__));
            jacobian_(1,0) = (((((((- foot_y *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  cos__q_HFE__)) + ( valuesGetter_lengths.getValue_haa_hfe() *  sin__q_HAA__));
            jacobian_(1,1) = (((((( foot_y *  sin__q_HAA__) *  sin__q_HFE__) - (( foot_x *  sin__q_HAA__) *  cos__q_HFE__)) *  sin__q_KFE__) + ((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) - (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  sin__q_HFE__));
            jacobian_(1,2) = ((((( foot_y *  sin__q_HAA__) *  sin__q_HFE__) - (( foot_x *  sin__q_HAA__) *  cos__q_HFE__)) *  sin__q_KFE__) + ((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));
            jacobian_(2,0) = (((((((- foot_y *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_x *  sin__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_x *  sin__q_HAA__) *  cos__q_HFE__) - (( foot_y *  sin__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  sin__q_HAA__) *  cos__q_HFE__)) - ( valuesGetter_lengths.getValue_haa_hfe() *  cos__q_HAA__));
            jacobian_(2,1) = (((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__)) + (( valuesGetter_lengths.getValue_upper_leg() *  cos__q_HAA__) *  sin__q_HFE__));
            jacobian_(2,2) = ((((( foot_x *  cos__q_HAA__) *  cos__q_HFE__) - (( foot_y *  cos__q_HAA__) *  sin__q_HFE__)) *  sin__q_KFE__) + (((( foot_y *  cos__q_HAA__) *  cos__q_HFE__) + (( foot_x *  cos__q_HAA__) *  sin__q_HFE__)) *  cos__q_KFE__));

        break;
    }
    return jacobian_;
}
}
}


iit::Aliengo::Jacobians::Type_fr_trunk_J_LF_foot::Type_fr_trunk_J_LF_foot(const Params_lengths& _lengths_values)
    : lengths_values(& _lengths_values)
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

const iit::Aliengo::Jacobians::Type_fr_trunk_J_LF_foot& iit::Aliengo::Jacobians::Type_fr_trunk_J_LF_foot::update(const JointState& jState) {
    static double sin__q_LF_HAA__;
    static double sin__q_LF_HFE__;
    static double sin__q_LF_KFE__;
    static double cos__q_LF_HAA__;
    static double cos__q_LF_HFE__;
    static double cos__q_LF_KFE__;
    
    sin__q_LF_HAA__ = std::sin( jState(LF_HAA));
    sin__q_LF_HFE__ = std::sin( jState(LF_HFE));
    sin__q_LF_KFE__ = std::sin( jState(LF_KFE));
    cos__q_LF_HAA__ = std::cos( jState(LF_HAA));
    cos__q_LF_HFE__ = std::cos( jState(LF_HFE));
    cos__q_LF_KFE__ = std::cos( jState(LF_KFE));
    
    (*this)(1,1) =  cos__q_LF_HAA__;
    (*this)(1,2) =  cos__q_LF_HAA__;
    (*this)(2,1) = - sin__q_LF_HAA__;
    (*this)(2,2) = - sin__q_LF_HAA__;
    (*this)(3,1) = (((( lengths_values -> foot_x *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( lengths_values -> foot_x *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - ( lengths_values -> upper_leg *  cos__q_LF_HFE__));
    (*this)(3,2) = ((( lengths_values -> foot_x *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - (( lengths_values -> foot_x *  cos__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(4,0) = (((((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) - ((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) - (( lengths_values -> upper_leg *  cos__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( lengths_values -> haa_hfe *  sin__q_LF_HAA__));
    (*this)(4,1) = ((((( lengths_values -> foot_x *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( lengths_values -> foot_x *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( lengths_values -> upper_leg *  sin__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(4,2) = (((( lengths_values -> foot_x *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( lengths_values -> foot_x *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    (*this)(5,0) = ((((((- lengths_values -> foot_x *  sin__q_LF_HAA__) *  sin__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( lengths_values -> foot_x *  sin__q_LF_HAA__) *  cos__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( lengths_values -> upper_leg *  sin__q_LF_HAA__) *  cos__q_LF_HFE__)) - ( lengths_values -> haa_hfe *  cos__q_LF_HAA__));
    (*this)(5,1) = ((((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__)) + (( lengths_values -> upper_leg *  cos__q_LF_HAA__) *  sin__q_LF_HFE__));
    (*this)(5,2) = (((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  cos__q_LF_HFE__) *  sin__q_LF_KFE__) + ((( lengths_values -> foot_x *  cos__q_LF_HAA__) *  sin__q_LF_HFE__) *  cos__q_LF_KFE__));
    return *this;
}
iit::Aliengo::Jacobians::Type_fr_trunk_J_RF_foot::Type_fr_trunk_J_RF_foot(const Params_lengths& _lengths_values)
    : lengths_values(& _lengths_values)
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

const iit::Aliengo::Jacobians::Type_fr_trunk_J_RF_foot& iit::Aliengo::Jacobians::Type_fr_trunk_J_RF_foot::update(const JointState& jState) {
    static double sin__q_RF_HAA__;
    static double sin__q_RF_HFE__;
    static double sin__q_RF_KFE__;
    static double cos__q_RF_HAA__;
    static double cos__q_RF_HFE__;
    static double cos__q_RF_KFE__;
    
    sin__q_RF_HAA__ = std::sin( jState(RF_HAA));
    sin__q_RF_HFE__ = std::sin( jState(RF_HFE));
    sin__q_RF_KFE__ = std::sin( jState(RF_KFE));
    cos__q_RF_HAA__ = std::cos( jState(RF_HAA));
    cos__q_RF_HFE__ = std::cos( jState(RF_HFE));
    cos__q_RF_KFE__ = std::cos( jState(RF_KFE));
    
    (*this)(1,1) =  cos__q_RF_HAA__;
    (*this)(1,2) =  cos__q_RF_HAA__;
    (*this)(2,1) =  sin__q_RF_HAA__;
    (*this)(2,2) =  sin__q_RF_HAA__;
    (*this)(3,1) = (((( lengths_values -> foot_x *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( lengths_values -> foot_x *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) - ( lengths_values -> upper_leg *  cos__q_RF_HFE__));
    (*this)(3,2) = ((( lengths_values -> foot_x *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) - (( lengths_values -> foot_x *  cos__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(4,0) = ((((((- lengths_values -> foot_x *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( lengths_values -> foot_x *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( lengths_values -> upper_leg *  cos__q_RF_HAA__) *  cos__q_RF_HFE__)) + ( lengths_values -> haa_hfe *  sin__q_RF_HAA__));
    (*this)(4,1) = (((((- lengths_values -> foot_x *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( lengths_values -> foot_x *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) - (( lengths_values -> upper_leg *  sin__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(4,2) = ((((- lengths_values -> foot_x *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) - ((( lengths_values -> foot_x *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    (*this)(5,0) = ((((((- lengths_values -> foot_x *  sin__q_RF_HAA__) *  sin__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( lengths_values -> foot_x *  sin__q_RF_HAA__) *  cos__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( lengths_values -> upper_leg *  sin__q_RF_HAA__) *  cos__q_RF_HFE__)) - ( lengths_values -> haa_hfe *  cos__q_RF_HAA__));
    (*this)(5,1) = ((((( lengths_values -> foot_x *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( lengths_values -> foot_x *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__)) + (( lengths_values -> upper_leg *  cos__q_RF_HAA__) *  sin__q_RF_HFE__));
    (*this)(5,2) = (((( lengths_values -> foot_x *  cos__q_RF_HAA__) *  cos__q_RF_HFE__) *  sin__q_RF_KFE__) + ((( lengths_values -> foot_x *  cos__q_RF_HAA__) *  sin__q_RF_HFE__) *  cos__q_RF_KFE__));
    return *this;
}
iit::Aliengo::Jacobians::Type_fr_trunk_J_LH_foot::Type_fr_trunk_J_LH_foot(const Params_lengths& _lengths_values)
    : lengths_values(& _lengths_values)
{
    (*this)(0,0) = - 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

const iit::Aliengo::Jacobians::Type_fr_trunk_J_LH_foot& iit::Aliengo::Jacobians::Type_fr_trunk_J_LH_foot::update(const JointState& jState) {
    static double sin__q_LH_HAA__;
    static double sin__q_LH_HFE__;
    static double sin__q_LH_KFE__;
    static double cos__q_LH_HAA__;
    static double cos__q_LH_HFE__;
    static double cos__q_LH_KFE__;
    
    sin__q_LH_HAA__ = std::sin( jState(LH_HAA));
    sin__q_LH_HFE__ = std::sin( jState(LH_HFE));
    sin__q_LH_KFE__ = std::sin( jState(LH_KFE));
    cos__q_LH_HAA__ = std::cos( jState(LH_HAA));
    cos__q_LH_HFE__ = std::cos( jState(LH_HFE));
    cos__q_LH_KFE__ = std::cos( jState(LH_KFE));
    
    (*this)(1,1) =  cos__q_LH_HAA__;
    (*this)(1,2) =  cos__q_LH_HAA__;
    (*this)(2,1) = - sin__q_LH_HAA__;
    (*this)(2,2) = - sin__q_LH_HAA__;
    (*this)(3,1) = (((( lengths_values -> foot_x *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( lengths_values -> foot_x *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - ( lengths_values -> upper_leg *  cos__q_LH_HFE__));
    (*this)(3,2) = ((( lengths_values -> foot_x *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - (( lengths_values -> foot_x *  cos__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(4,0) = (((((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) - ((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) - (( lengths_values -> upper_leg *  cos__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( lengths_values -> haa_hfe *  sin__q_LH_HAA__));
    (*this)(4,1) = ((((( lengths_values -> foot_x *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( lengths_values -> foot_x *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( lengths_values -> upper_leg *  sin__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(4,2) = (((( lengths_values -> foot_x *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( lengths_values -> foot_x *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    (*this)(5,0) = ((((((- lengths_values -> foot_x *  sin__q_LH_HAA__) *  sin__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( lengths_values -> foot_x *  sin__q_LH_HAA__) *  cos__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( lengths_values -> upper_leg *  sin__q_LH_HAA__) *  cos__q_LH_HFE__)) - ( lengths_values -> haa_hfe *  cos__q_LH_HAA__));
    (*this)(5,1) = ((((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__)) + (( lengths_values -> upper_leg *  cos__q_LH_HAA__) *  sin__q_LH_HFE__));
    (*this)(5,2) = (((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  cos__q_LH_HFE__) *  sin__q_LH_KFE__) + ((( lengths_values -> foot_x *  cos__q_LH_HAA__) *  sin__q_LH_HFE__) *  cos__q_LH_KFE__));
    return *this;
}
iit::Aliengo::Jacobians::Type_fr_trunk_J_RH_foot::Type_fr_trunk_J_RH_foot(const Params_lengths& _lengths_values)
    : lengths_values(& _lengths_values)
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 0;
    (*this)(3,0) = 0;
}

const iit::Aliengo::Jacobians::Type_fr_trunk_J_RH_foot& iit::Aliengo::Jacobians::Type_fr_trunk_J_RH_foot::update(const JointState& jState) {
    static double sin__q_RH_HAA__;
    static double sin__q_RH_HFE__;
    static double sin__q_RH_KFE__;
    static double cos__q_RH_HAA__;
    static double cos__q_RH_HFE__;
    static double cos__q_RH_KFE__;
    
    sin__q_RH_HAA__ = std::sin( jState(RH_HAA));
    sin__q_RH_HFE__ = std::sin( jState(RH_HFE));
    sin__q_RH_KFE__ = std::sin( jState(RH_KFE));
    cos__q_RH_HAA__ = std::cos( jState(RH_HAA));
    cos__q_RH_HFE__ = std::cos( jState(RH_HFE));
    cos__q_RH_KFE__ = std::cos( jState(RH_KFE));
    
    (*this)(1,1) =  cos__q_RH_HAA__;
    (*this)(1,2) =  cos__q_RH_HAA__;
    (*this)(2,1) =  sin__q_RH_HAA__;
    (*this)(2,2) =  sin__q_RH_HAA__;
    (*this)(3,1) = (((( lengths_values -> foot_x *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( lengths_values -> foot_x *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) - ( lengths_values -> upper_leg *  cos__q_RH_HFE__));
    (*this)(3,2) = ((( lengths_values -> foot_x *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) - (( lengths_values -> foot_x *  cos__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(4,0) = ((((((- lengths_values -> foot_x *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( lengths_values -> foot_x *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( lengths_values -> upper_leg *  cos__q_RH_HAA__) *  cos__q_RH_HFE__)) + ( lengths_values -> haa_hfe *  sin__q_RH_HAA__));
    (*this)(4,1) = (((((- lengths_values -> foot_x *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( lengths_values -> foot_x *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) - (( lengths_values -> upper_leg *  sin__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(4,2) = ((((- lengths_values -> foot_x *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) - ((( lengths_values -> foot_x *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    (*this)(5,0) = ((((((- lengths_values -> foot_x *  sin__q_RH_HAA__) *  sin__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( lengths_values -> foot_x *  sin__q_RH_HAA__) *  cos__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( lengths_values -> upper_leg *  sin__q_RH_HAA__) *  cos__q_RH_HFE__)) - ( lengths_values -> haa_hfe *  cos__q_RH_HAA__));
    (*this)(5,1) = ((((( lengths_values -> foot_x *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( lengths_values -> foot_x *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__)) + (( lengths_values -> upper_leg *  cos__q_RH_HAA__) *  sin__q_RH_HFE__));
    (*this)(5,2) = (((( lengths_values -> foot_x *  cos__q_RH_HAA__) *  cos__q_RH_HFE__) *  sin__q_RH_KFE__) + ((( lengths_values -> foot_x *  cos__q_RH_HAA__) *  sin__q_RH_HFE__) *  cos__q_RH_KFE__));
    return *this;
}
