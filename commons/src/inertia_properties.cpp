#include "iit/robots/aliengo/inertia_properties.h"

using namespace std;
using namespace iit::rbd;

//HACK: INERTIA PROPERTIES FOR ALIENGO
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

    com_LF_hipassembly = iit::rbd::Vector3d(0.000015,-0.015144,0.022191); //updated with Aliengo
    tensor_LF_hipassembly.fill(
    		1.993,
			com_LF_hipassembly,
			Utils::buildInertiaTensor(
					0.005586944,
					0.004907517,
					0.002903894,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_LF_upperleg = iit::rbd::Vector3d(0.048199,0.005607,-0.003877);  //updated with Aliengo
    tensor_LF_upperleg.fill(
    		0.639,
			com_LF_upperleg,
			Utils::buildInertiaTensor(
					0.000369811,
					0.005666803,
					0.005847229,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_LF_lowerleg = iit::rbd::Vector3d(0.142518,-0.002781,0.000063);  //updated with Aliengo
    tensor_LF_lowerleg.fill(
                0.207+0.06,
			com_LF_lowerleg,
			Utils::buildInertiaTensor(
					0.000039188,
					0.006341369,
					0.006355157,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RF_hipassembly = iit::rbd::Vector3d(0.000015,-0.015144,-0.022191); //updated with Aliengo
    tensor_RF_hipassembly.fill(
    		1.993,
			com_RF_hipassembly,
			Utils::buildInertiaTensor(
					0.005586944,
					0.004907517,
					0.002903894,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RF_upperleg = iit::rbd::Vector3d(0.048199,0.005607,0.003877);  //updated with Aliengo
    tensor_RF_upperleg.fill(
    		0.639,
			com_RF_upperleg,
			Utils::buildInertiaTensor(
					0.000369811,
					0.005666803,
					0.005847229,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RF_lowerleg = iit::rbd::Vector3d(0.142518,-0.002781,0.000063);  //updated with Aliengo
    tensor_RF_lowerleg.fill(
                0.207+0.06,
			com_RF_lowerleg,
			Utils::buildInertiaTensor(
					0.000039188,
					0.006341369,
					0.006355157,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_LH_hipassembly = iit::rbd::Vector3d(0.000015,-0.015144,0.022191); //updated with Aliengo
    tensor_LH_hipassembly.fill(
    		1.993,
			com_LH_hipassembly,
			Utils::buildInertiaTensor(
					0.005586944,
					0.004907517,
					0.002903894,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_LH_upperleg = iit::rbd::Vector3d(0.048199,0.005607,-0.003877);  //updated with Aliengo
    tensor_LH_upperleg.fill(
    		0.639,
			com_LH_upperleg,
			Utils::buildInertiaTensor(
					0.000369811,
					0.005666803,
					0.005847229,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_LH_lowerleg = iit::rbd::Vector3d(0.142518,-0.002781,0.000063);  //updated with Aliengo
    tensor_LH_lowerleg.fill(
                0.207+0.06,
			com_LH_lowerleg,
			Utils::buildInertiaTensor(
					0.000039188,
					0.006341369,
					0.006355157,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RH_hipassembly = iit::rbd::Vector3d(0.000015,-0.015144,-0.022191); //updated with Aliengo
    tensor_RH_hipassembly.fill(
    		1.993,
			com_RH_hipassembly,
			Utils::buildInertiaTensor(
					0.005586944,
					0.004907517,
					0.002903894,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RH_upperleg = iit::rbd::Vector3d(0.048199,0.005607,0.003877);  //updated with Aliengo
    tensor_RH_upperleg.fill(
    		0.639,
			com_RH_upperleg,
			Utils::buildInertiaTensor(
					0.000369811,
					0.005666803,
					0.005847229,
                                        0.0,
                                        0.0,
                                        0.0) );

    com_RH_lowerleg = iit::rbd::Vector3d(0.142518,-0.002781,0.000063);  //updated with Aliengo
    tensor_RH_lowerleg.fill(
                0.207+0.06,
			com_RH_lowerleg,
			Utils::buildInertiaTensor(
					0.000039188,
					0.006341369,
					0.006355157,
                                        0.0,
                                        0.0,
                                        0.0) );

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
    throw std::range_error("Leg ID not known");
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
    throw std::range_error("Leg ID not known");
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
    throw std::range_error("Leg ID not known");
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
    throw std::range_error("Leg ID not known");
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
    throw std::range_error("Leg ID not known");
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
    throw std::range_error("Leg ID not known");
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
