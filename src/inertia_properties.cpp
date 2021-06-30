#include "iit/robots/aliengo/inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::Aliengo::dyn::InertiaProperties::InertiaProperties(const dog::KinDynParams& pGetter)
    : paramsGetter(pGetter), homogeneus_transforms(pGetter)
{
    com_trunk = iit::rbd::Vector3d(paramsGetter.getValue_trunk_com_x(),paramsGetter.getValue_trunk_com_y(),paramsGetter.getValue_trunk_com_z());
    tensor_trunk.fill(
        paramsGetter.getValue_trunk_mass(),
        com_trunk,
        Utils::buildInertiaTensor(
                paramsGetter.getValue_trunk_Ix(),
                paramsGetter.getValue_trunk_Iy(),
                paramsGetter.getValue_trunk_Iz(),
                paramsGetter.getValue_trunk_Ixy(),
                paramsGetter.getValue_trunk_Ixz(),
                paramsGetter.getValue_trunk_Iyz()) );

    com_LF_hipassembly = iit::rbd::Vector3d(-0.010027,-0.010608,0.003644);
    tensor_LF_hipassembly.fill(
        4.973275,
        com_LF_hipassembly,
        Utils::buildInertiaTensor(
                0.011857,
                0.016334,
                0.017923,
                -6.3E-5,
                4.4E-5,
                3.2E-5) );

    com_LF_upperleg = iit::rbd::Vector3d(0.107987,-0.011459,-0.002121);
    tensor_LF_upperleg.fill(
        5.877927,
        com_LF_upperleg,
        Utils::buildInertiaTensor(
                0.01723,
                0.144308,
                0.148583,
                -0.00541,
                0.00255,
                -9.7E-5) );

    com_LF_lowerleg = iit::rbd::Vector3d(0.149302,-0.003801,-6.24E-4);
    tensor_LF_lowerleg.fill(
        1.291954,
        com_LF_lowerleg,
        Utils::buildInertiaTensor(
                0.001107,
                0.058015,
                0.05859,
                -0.001305,
                -1.0E-6,
                3.0E-6) );

    com_RF_hipassembly = iit::rbd::Vector3d(-0.010027,-0.010608,0.003644);
    tensor_RF_hipassembly.fill(
        4.973275,
        com_RF_hipassembly,
        Utils::buildInertiaTensor(
                0.011857,
                0.016334,
                0.017923,
                -6.3E-5,
                4.4E-5,
                3.2E-5) );

    com_RF_upperleg = iit::rbd::Vector3d(0.107987,-0.011459,-0.002121);
    tensor_RF_upperleg.fill(
        5.877927,
        com_RF_upperleg,
        Utils::buildInertiaTensor(
                0.01723,
                0.144308,
                0.148583,
                -0.00541,
                0.00255,
                -9.7E-5) );

    com_RF_lowerleg = iit::rbd::Vector3d(0.149302,-0.003801,-6.24E-4);
    tensor_RF_lowerleg.fill(
        1.291954,
        com_RF_lowerleg,
        Utils::buildInertiaTensor(
                0.001107,
                0.058015,
                0.05859,
                -0.001305,
                -1.0E-6,
                3.0E-6) );

    com_LH_hipassembly = iit::rbd::Vector3d(-0.010027,-0.010608,0.003644);
    tensor_LH_hipassembly.fill(
        4.973275,
        com_LH_hipassembly,
        Utils::buildInertiaTensor(
                0.011857,
                0.016334,
                0.017923,
                -6.3E-5,
                4.4E-5,
                3.2E-5) );

    com_LH_upperleg = iit::rbd::Vector3d(0.107987,-0.011459,-0.002121);
    tensor_LH_upperleg.fill(
        5.877927,
        com_LH_upperleg,
        Utils::buildInertiaTensor(
                0.01723,
                0.144308,
                0.148583,
                -0.00541,
                0.00255,
                -9.7E-5) );

    com_LH_lowerleg = iit::rbd::Vector3d(0.149302,-0.003801,-6.24E-4);
    tensor_LH_lowerleg.fill(
        1.291954,
        com_LH_lowerleg,
        Utils::buildInertiaTensor(
                0.001107,
                0.058015,
                0.05859,
                -0.001305,
                -1.0E-6,
                3.0E-6) );

    com_RH_hipassembly = iit::rbd::Vector3d(-0.010027,-0.010608,0.003644);
    tensor_RH_hipassembly.fill(
        4.973275,
        com_RH_hipassembly,
        Utils::buildInertiaTensor(
                0.011857,
                0.016334,
                0.017923,
                -6.3E-5,
                4.4E-5,
                3.2E-5) );

    com_RH_upperleg = iit::rbd::Vector3d(0.107987,-0.011459,-0.002121);
    tensor_RH_upperleg.fill(
        5.877927,
        com_RH_upperleg,
        Utils::buildInertiaTensor(
                0.01723,
                0.144308,
                0.148583,
                -0.00541,
                0.00255,
                -9.7E-5) );

    com_RH_lowerleg = iit::rbd::Vector3d(0.149302,-0.003801,-6.24E-4);
    tensor_RH_lowerleg.fill(
        1.291954,
        com_RH_lowerleg,
        Utils::buildInertiaTensor(
                0.001107,
                0.058015,
                0.05859,
                -0.001305,
                -1.0E-6,
                3.0E-6) );

}

void iit::Aliengo::dyn::InertiaProperties::updateParameters() {
    com_trunk = iit::rbd::Vector3d(paramsGetter.getValue_trunk_com_x(),paramsGetter.getValue_trunk_com_y(),paramsGetter.getValue_trunk_com_z());
        tensor_trunk.fill(
            paramsGetter.getValue_trunk_mass(),
            com_trunk,
            Utils::buildInertiaTensor(
                    paramsGetter.getValue_trunk_Ix(),
                    paramsGetter.getValue_trunk_Iy(),
                    paramsGetter.getValue_trunk_Iz(),
                    paramsGetter.getValue_trunk_Ixy(),
                    paramsGetter.getValue_trunk_Ixz(),
                    paramsGetter.getValue_trunk_Iyz()));
}

const iit::rbd::Vector3d& iit::Aliengo::dyn::InertiaProperties::getTrunkCOM() {
    updateParameters();
    return this->getCOM_trunk();
}

double iit::Aliengo::dyn::InertiaProperties::getTrunkMass() {
    updateParameters();
    return this->getMass_trunk();
}
double iit::Aliengo::dyn::InertiaProperties::getHipAssemblyMass(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getMass_LF_hipassembly();
    case dog::RF:
        return getMass_RF_hipassembly();
    case dog::LH:
        return getMass_LH_hipassembly();
    case dog::RH:
        return getMass_RH_hipassembly();
    }
}
double iit::Aliengo::dyn::InertiaProperties::getUpperLegMass(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getMass_LF_upperleg();
    case dog::RF:
        return getMass_RF_upperleg();
    case dog::LH:
        return getMass_LH_upperleg();
    case dog::RH:
        return getMass_RH_upperleg();
    }
}
double iit::Aliengo::dyn::InertiaProperties::getLowerLegMass(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getMass_LF_lowerleg();
    case dog::RF:
        return getMass_RF_lowerleg();
    case dog::LH:
        return getMass_LH_lowerleg();
    case dog::RH:
        return getMass_RH_lowerleg();
    }
}
iit::rbd::Vector3d iit::Aliengo::dyn::InertiaProperties::getHipAssemblyCOM(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getCOM_LF_hipassembly();
    case dog::RF:
        return getCOM_RF_hipassembly();
    case dog::LH:
        return getCOM_LH_hipassembly();
    case dog::RH:
        return getCOM_RH_hipassembly();
    }
}
iit::rbd::Vector3d iit::Aliengo::dyn::InertiaProperties::getUpperLegCOM(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getCOM_LF_upperleg();
    case dog::RF:
        return getCOM_RF_upperleg();
    case dog::LH:
        return getCOM_LH_upperleg();
    case dog::RH:
        return getCOM_RH_upperleg();
    }
}
iit::rbd::Vector3d iit::Aliengo::dyn::InertiaProperties::getLowerLegCOM(const dog::LegID& leg) const{
    switch(leg){
    case dog::LF:
        return getCOM_LF_lowerleg();
    case dog::RF:
        return getCOM_RF_lowerleg();
    case dog::LH:
        return getCOM_LH_lowerleg();
    case dog::RH:
        return getCOM_RH_lowerleg();
    }
}
iit::rbd::Vector3d iit::Aliengo::dyn::InertiaProperties::getWholeBodyCOM(const dog::JointState &q){
    homogeneus_transforms.fr_trunk_X_fr_LF_hipassembly(q);
    homogeneus_transforms.fr_trunk_X_fr_RF_hipassembly(q);
    homogeneus_transforms.fr_trunk_X_fr_LH_hipassembly(q);
    homogeneus_transforms.fr_trunk_X_fr_RH_hipassembly(q);
    homogeneus_transforms.fr_LF_hipassembly_X_fr_LF_upperleg(q);
    homogeneus_transforms.fr_LF_upperleg_X_fr_LF_lowerleg(q);
    homogeneus_transforms.fr_RF_hipassembly_X_fr_RF_upperleg(q);
    homogeneus_transforms.fr_RF_upperleg_X_fr_RF_lowerleg(q);
    homogeneus_transforms.fr_LH_hipassembly_X_fr_LH_upperleg(q);
    homogeneus_transforms.fr_LH_upperleg_X_fr_LH_lowerleg(q);
    homogeneus_transforms.fr_RH_hipassembly_X_fr_RH_upperleg(q);
    homogeneus_transforms.fr_RH_upperleg_X_fr_RH_lowerleg(q);
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());
    tmpSum += this->getTrunkCOM() * this->getTrunkMass();
    static HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_LF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_LH_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RH_HAA_chain;
    base_X_LF_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_LF_hipassembly;
    tmpSum += this->getMass_LF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_hipassembly()));
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * homogeneus_transforms.fr_LF_hipassembly_X_fr_LF_upperleg;
    tmpSum += this->getMass_LF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_upperleg()));
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * homogeneus_transforms.fr_LF_upperleg_X_fr_LF_lowerleg;
    tmpSum += this->getMass_LF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, this->getCOM_LF_lowerleg()));
    base_X_RF_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_RF_hipassembly;
    tmpSum += this->getMass_RF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_hipassembly()));
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * homogeneus_transforms.fr_RF_hipassembly_X_fr_RF_upperleg;
    tmpSum += this->getMass_RF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_upperleg()));
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * homogeneus_transforms.fr_RF_upperleg_X_fr_RF_lowerleg;
    tmpSum += this->getMass_RF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, this->getCOM_RF_lowerleg()));
    base_X_LH_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_LH_hipassembly;
    tmpSum += this->getMass_LH_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_hipassembly()));
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * homogeneus_transforms.fr_LH_hipassembly_X_fr_LH_upperleg;
    tmpSum += this->getMass_LH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_upperleg()));
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * homogeneus_transforms.fr_LH_upperleg_X_fr_LH_lowerleg;
    tmpSum += this->getMass_LH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, this->getCOM_LH_lowerleg()));
    base_X_RH_HAA_chain = tmpX * homogeneus_transforms.fr_trunk_X_fr_RH_hipassembly;
    tmpSum += this->getMass_RH_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_hipassembly()));
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * homogeneus_transforms.fr_RH_hipassembly_X_fr_RH_upperleg;
    tmpSum += this->getMass_RH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_upperleg()));
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * homogeneus_transforms.fr_RH_upperleg_X_fr_RH_lowerleg;
    tmpSum += this->getMass_RH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, this->getCOM_RH_lowerleg()));
    return tmpSum / this->getTotalMass();
}
