/*
 * PIDControl.h
 *
 *  Created on: May 22, 2013
 *      Author: mfrigerio
 */

#ifndef _IIT_ALIENGO_COMMONS_PIDCONTROL_H_
#define _IIT_ALIENGO_COMMONS_PIDCONTROL_H_

#include <iit/commons/dog/declarations.h>
#include <iit/commons/dog/joint_data_map.h>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/control.h>

namespace iit {
namespace Aliengo {


typedef commons::ctrl::pid_errors pid_errors; // to cope with a possible change in iit:: ...
typedef commons::ctrl::pid_gains  pid_gains;
typedef commons::ctrl::pid_terms  pid_terms;

typedef dog::JointDataMap<pid_errors> JointsPIDerrors;
typedef dog::JointDataMap<pid_gains>  JointsPIDgains;
typedef dog::JointDataMap<pid_terms>  JointsPIDterms;
typedef dog::JointIdentifiers JointIdentifiers;




class JointsPIDControl
{
public:
    typedef dog::JointDataMap<double>  References;
    typedef dog::JointDataMap<double>  States;
public:
    JointsPIDControl(double ts, double tf);

    double getTs() const;
    double getTf() const;

    void setGains(const JointsPIDgains& gains);
    const JointsPIDgains& getGains() const;

    const JointsPIDerrors& getErrors() const;
    const JointsPIDterms&  getTerms()  const;

    void pid(
            const References& ref,
            const States&     actual);

private:
    void singleJointPID(JointIdentifiers j, double ref, double actual);
private:
    commons::ctrl::PIDControl controller;
    JointsPIDerrors errors;
    JointsPIDgains  gains;
    JointsPIDterms  terms;
};


inline JointsPIDControl::JointsPIDControl(double ts, double tf)
    : controller(ts, tf), errors(0.0), gains(0.0), terms(0.0)
{
}


inline double JointsPIDControl::getTs() const {
    return controller.getTs();
}
inline double JointsPIDControl::getTf() const {
    return controller.getTf();
}

inline void JointsPIDControl::setGains(const JointsPIDgains& g) {
    gains = g;
}

inline const JointsPIDgains& JointsPIDControl::getGains() const {
    return gains;
}

inline const JointsPIDerrors& JointsPIDControl::getErrors() const {
    return errors;
}

inline const JointsPIDterms& JointsPIDControl::getTerms() const {
    return terms;
}

inline void JointsPIDControl::singleJointPID(
        JointIdentifiers j,
        double ref,
        double actual)
{
    errors[j].current = ref - actual;
    controller.pid(gains[j], errors[j], terms[j]);
    errors[j].updatePrevious();
}

}
}



#endif
