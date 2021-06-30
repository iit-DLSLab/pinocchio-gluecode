/*
 * fbl.cpp
 *
 *  Created on: June 26, 2018
 *      Author: Gennaro Raiola
 */

#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/four_bar_linkage.h>
#include <iit/robots/aliengo/mechanical_constants.h>
#include <Eigen/Core>
#include <gtest/gtest.h>

using namespace iit::Aliengo;
using namespace iit::dog;
using namespace iit::commons;

/// Angles limits for Aliengo, directly taken from the urdf
/// <xacro:property name="kfe_max" value="165"/>  radiants: 2.8797932657906435
/// <xacro:property name="kfe_min" value="32"/>  radiants: 0.5585053606381855

TEST(FBLTest,compare_with_polynomial_approx)
{

    FBLConsts fblConsts;
    fblConsts.akh = fbl::akh;
    fblConsts.cj =  fbl::cj;
    fblConsts.jk = fbl::jk;
    fblConsts.ak = fbl::ak;
    fblConsts.bk = fbl::bk;
    fblConsts.ar = fbl::ar;
    fblConsts.br = fbl::br;
    fblConsts.angle_KB_lleg_xaxis = fbl::angle_KB_lleg_xaxis;
    FBLStatus fbl(fblConsts);

    Eigen::VectorXd knee_angles = Eigen::VectorXd::LinSpaced(100, -2.8797932657906435 , -0.5585053606381855); //Number of samples, min angle, max angle

    for(int i=0; i<knee_angles.size(); i++) {

        //std::cout << knee_angles(i) <<std::endl;
        fbl.setAngle(knee_angles(i));
        fbl.computeNumericJacobian(knee_angles(i));

        //std::cout << "extensions " << fbl.extension; // 2
        double lever_arm_polyn = 0.00006412*pow(knee_angles(i),6) + 0.0002046*pow(knee_angles(i),5) -0.0006317*pow(knee_angles(i),4)
                          -0.0007159*pow(knee_angles(i),3) +0.00008877*pow(knee_angles(i),2) -0.01217*knee_angles(i)  +  0.01663;

        //std::cout<<"Numeric: "<<  fbl.jac_num<<" Poly: " <<lever_arm_polyn<<" err :" <<fabs(fbl.jac_num - lever_arm_polyn)<<std::endl;
        //std::cout << std::endl;

        ASSERT_NEAR(fbl.jac_num,lever_arm_polyn,1e-4);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
