#ifndef IIT_ALIENGO_DEFAULT_PARAMS_GETTER_H_
#define IIT_ALIENGO_DEFAULT_PARAMS_GETTER_H_

#include "dog/kin_dyn_params.h"

#include "robcogen/kinematics_parameters.h"
#include "robcogen/dynamics_parameters.h"

namespace iit {
namespace Aliengo {

/**
 * @brief The DefaultParamsGetter class
 * @date October 9th, 2013
 * @author Andreea Radulescu (andreea.radulescu@iit.it)
 * @author Marco Camurri (marco.camurri@iit.it)
 */
class DefaultParamsGetter : public dog::KinDynParams
{
    public:
        DefaultParamsGetter();
        virtual ~DefaultParamsGetter() {}
    public:
        void resetDefaults() {

            inertias.trunk_mass = 9.041; 
            inertias.trunk_com_x = 0.008465; // Updated to Aliengo
            inertias.trunk_com_y = 0.004045; // Updated to Aliengo
            inertias.trunk_com_z =-0.000763; // Updated to Aliengo
            inertias.trunk_Ix = 0.033260231; // Updated to Aliengo
            inertias.trunk_Iy = 0.16117211; // Updated to Aliengo
            inertias.trunk_Iz  = 0.17460442; // Updated to Aliengo
            inertias.trunk_Ixy = -0.000451628; // Updated to Aliengo
            inertias.trunk_Ixz = 0.000487603; // Updated to Aliengo
            inertias.trunk_Iyz = 0.000048356; // Updated to Aliengo

            value.haa_y = 0.051;  // Updated to Aliengo: <xacro:property name="leg_offset_y" value="0.051"/>
            value.haa_x = 0.2399; // Updated to Aliengo: <xacro:property name="leg_offset_x" value="0.2399"/>
            value.haa_hfe = 0.083; // Updated to Aliengo: <xacro:property name="hip_offset" value="0.083"/>
            value.upper_leg = 0.25; // Updated to Aliengo: <xacro:property name="thigh_length" value="0.25"/>
            value.foot_x = 0.25; // Updated to Aliengo: <xacro:property name="calf_length" value="0.25"/>

	    // there is no parameter to assign trunk_offset_z
	    // <xacro:property name="trunk_offset_z" value="0.01675"/>
        }

        double getValue_foot_x() const {
            return value.foot_x;
        }

        double getValue_robot_total_mass() const {
            return inertias.robot_mass;
        }
        //getters
        double getValue_trunk_mass() const {
            return inertias.trunk_mass;
        }
        double getValue_trunk_com_x() const {
            return inertias.trunk_com_x;
        }
        double getValue_trunk_com_y() const {
            return inertias.trunk_com_y;
        }
        double getValue_trunk_com_z() const {
            return inertias.trunk_com_z;
        }
        double getValue_trunk_Ix() const {
            return inertias.trunk_Ix;
        }
        double getValue_trunk_Iy() const {
            return inertias.trunk_Iy;
        }
        double getValue_trunk_Iz() const {
            return inertias.trunk_Iz;
        }
        double getValue_trunk_Ixy() const {
            return inertias.trunk_Ixy;
        }
        double getValue_trunk_Ixz() const {
            return inertias.trunk_Ixz;
        }
        double getValue_trunk_Iyz() const {
            return inertias.trunk_Iyz;
        }
        //setters
        virtual void setValue_robot_total_mass(double val){
             inertias.robot_mass =val;
        }
        virtual void setValue_trunk_mass(double val)  {
            inertias.trunk_mass =val;
        }
        virtual void setValue_trunk_com_x(double val)  {
            inertias.trunk_com_x =val;
        }
        virtual void setValue_trunk_com_y(double val)  {
            inertias.trunk_com_y =val;
        }
        virtual void setValue_trunk_com_z(double val)  {
            inertias.trunk_com_z =val;
        }
        virtual void setValue_trunk_Ix(double val)  {
            inertias.trunk_Ix =val;
        }
        virtual void setValue_trunk_Iy(double val)  {
            inertias.trunk_Iy =val;
        }
        virtual void setValue_trunk_Iz(double val)  {
            inertias.trunk_Iz =val;
        }
        virtual void setValue_trunk_Ixy(double val)  {
            inertias.trunk_Ixy =val;
        }
        virtual void setValue_trunk_Ixz(double val)  {
            inertias.trunk_Ixz =val;
        }
        virtual void setValue_trunk_Iyz(double val)  {
            inertias.trunk_Iyz =val;
        }

        inline void setValue_foot_x(double val);


        inline double getValue_haa_x() const;


        inline void setValue_haa_x(double val);


        inline double getValue_haa_y() const;

        inline double getValue_haa_z() const;


        inline void setValue_haa_y(double val);


        inline double getValue_haa_hfe() const;


        inline void setValue_haa_hfe(double val);

        inline double getValue_upper_leg() const;


        inline void setValue_upper_leg(double val);

        inline double getValue_lower_leg() const;


        inline void setValue_lower_leg(double val);

        inline const iit::Aliengo::Params_lengths& getLengths() const {
            return value;
        }

        inline void setLengths(const iit::Aliengo::Params_lengths& val)
        {
            value = val;
        }

        double getValue_LF_shin() const {
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_RF_shin() const {
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_LH_shin() const {
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            return getValue_foot_x();
        }

        double getValue_RH_shin() const {
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            return getValue_foot_x();
        }

        void setValue_LF_shin(double value){
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            std::cout << value << std::endl;   // TODO: Not used. Created to remove warning
        }
        void setValue_RF_shin(double value){
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            std::cout << value << std::endl;   // TODO: Not used. Created to remove warning
        }
        void setValue_LH_shin(double value){
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            std::cout << value << std::endl;   // TODO: Not used. Created to remove warning
        }
        void setValue_RH_shin(double value){
            // TODO
            // the shin frame is not implemented in Aliengo, returning
            // the foot one
            std::cout << value << std::endl;   // TODO: Not used. Created to remove warning
        }

        /**
         * @return the whole set of the current values of the inertia parameters
         */
        const dyn::RuntimeInertiaParams& getInertiaParams() const {
            return inertias;
        }
        /**
         * Sets new values for all the inertia parameters
         */
        virtual void setInertiaParams(const dyn::RuntimeInertiaParams& newp) {
            inertias = newp;
        }

    private:
        Params_lengths value;
        dyn::RuntimeInertiaParams inertias;
};

inline DefaultParamsGetter::DefaultParamsGetter()
{
    resetDefaults();
}




inline void DefaultParamsGetter::setValue_foot_x(double val)
{
    value.foot_x = val;
}


inline double DefaultParamsGetter::getValue_haa_x() const
{
    return value.haa_x;
}

inline double DefaultParamsGetter::getValue_haa_z() const
{
    return 0;
}


inline void DefaultParamsGetter::setValue_haa_x(double val)
{
    value.haa_x = val;
}


inline double DefaultParamsGetter::getValue_haa_y() const
{
    return value.haa_y;
}


inline void DefaultParamsGetter::setValue_haa_y(double val)
{
    value.haa_y = val;
}


inline double DefaultParamsGetter::getValue_haa_hfe() const
{
    return value.haa_hfe;
}


inline void DefaultParamsGetter::setValue_haa_hfe(double val)
{
    value.haa_hfe = val;
}

inline double DefaultParamsGetter::getValue_upper_leg() const
{
    return value.upper_leg;
}


inline void DefaultParamsGetter::setValue_upper_leg(double val)
{
    value.upper_leg = val;
}

inline double DefaultParamsGetter::getValue_lower_leg() const
{
    return value.foot_x;
}


inline void DefaultParamsGetter::setValue_lower_leg(double val)
{
    value.foot_x = val;
}

}
}



#endif

