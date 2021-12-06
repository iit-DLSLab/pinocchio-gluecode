#include "aliengo_leg.hpp"

//TODO: READ FROM URDF
// Maybe the link names can be changed

namespace aliengolib
{
    CrexLeg::CrexLeg(const std::string &name,
                        const std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> &joints,
                        const std::array<std::shared_ptr<robotlib::Link>, NLINKS> &links)
        : Leg<NJOINTS, NLINKS>(name, joints, links),
            jointMap({//joint name, parent name, child name
                    {"LF_HAA", std::make_pair("TRUNK", "LF_ASSEMBLY")},
                    {"LF_HFE", std::make_pair("LF_ASSEMBLY", "LF_UPPERLEG")},
                    {"LF_KFE", std::make_pair("LF_UPPERLEG", "LF_LOWERLEG")},
                    {"RF_HAA", std::make_pair("TRUNK", "RF_ASSEMBLY")},
                    {"RF_HFE", std::make_pair("RF_ASSEMBLY", "RF_UPPERLEG")},
                    {"RF_KFE", std::make_pair("RF_UPPERLEG", "RF_LOWERLEG")},
                    {"LC_HAA", std::make_pair("TRUNK", "LC_ASSEMBLY")},
                    {"LC_HFE", std::make_pair("LC_ASSEMBLY", "LC_UPPERLEG")},
                    {"LC_KFE", std::make_pair("LC_UPPERLEG", "LC_LOWERLEG")},
                    {"RC_HAA", std::make_pair("TRUNK", "RC_ASSEMBLY")},
                    {"RC_HFE", std::make_pair("RC_ASSEMBLY", "RC_UPPERLEG")},
                    {"RC_KFE", std::make_pair("RC_UPPERLEG", "RC_LOWERLEG")},
                    {"LH_HAA", std::make_pair("TRUNK", "LH_ASSEMBLY")},
                    {"LH_HFE", std::make_pair("LH_ASSEMBLY", "LH_UPPERLEG")},
                    {"LH_KFE", std::make_pair("LH_UPPERLEG", "LH_LOWERLEG")},
                    {"RH_HAA", std::make_pair("TRUNK", "RH_ASSEMBLY")},
                    {"RH_HFE", std::make_pair("RH_ASSEMBLY", "RH_UPPERLEG")},
                    {"RH_KFE", std::make_pair("RH_UPPERLEG", "RH_LOWERLEG")}}),
            linkMap({//link name, parent name, child name
                    {"LF_ASSEMBLY", std::make_pair("LF_HAA", "LF_HFE")},
                    {"LF_UPPERLEG", std::make_pair("LF_HFE", "LF_KFE")},
                    {"LF_LOWERLEG", std::make_pair("LF_KFE", "")},
                    {"RF_ASSEMBLY", std::make_pair("RF_HAA", "RF_HFE")},
                    {"RF_UPPERLEG", std::make_pair("RF_HFE", "RF_KFE")},
                    {"RF_LOWERLEG", std::make_pair("RF_KFE", "")},
                    {"LC_ASSEMBLY", std::make_pair("LC_HAA", "LC_HFE")},
                    {"LC_UPPERLEG", std::make_pair("LC_HFE", "LC_KFE")},
                    {"LC_LOWERLEG", std::make_pair("LC_KFE", "")},
                    {"RC_ASSEMBLY", std::make_pair("RC_HAA", "RC_HFE")},
                    {"RC_UPPERLEG", std::make_pair("RC_HFE", "RC_KFE")},
                    {"RC_LOWERLEG", std::make_pair("RC_KFE", "")},
                    {"LH_ASSEMBLY", std::make_pair("LH_HAA", "LH_HFE")},
                    {"LH_UPPERLEG", std::make_pair("LH_HFE", "LH_KFE")},
                    {"LH_LOWERLEG", std::make_pair("LH_KFE", "")},
                    {"RH_ASSEMBLY", std::make_pair("RH_HAA", "RH_HFE")},
                    {"RH_UPPERLEG", std::make_pair("RH_HFE", "RH_KFE")},
                    {"RH_LOWERLEG", std::make_pair("RH_KFE", "")}}){};
    CrexLeg::~CrexLeg(){};

    const std::string CrexLeg::jointToChildName(const std::shared_ptr<robotlib::Joint> joint) const
    {
        const std::string joint_name = joint->getName();
        const std::string child_name = jointMap.find(joint_name)->second.second;
        return child_name;
    };
    const std::string CrexLeg::jointToParentName(const std::shared_ptr<robotlib::Joint> joint) const
    {
        const std::string joint_name = joint->getName();
        const std::string parent_name = jointMap.find(joint_name)->second.first;
        return parent_name;
    };

    const std::string CrexLeg::linkToChildName(const std::shared_ptr<robotlib::Link> link) const
    {
        const std::string link_name = link->getName();
        const std::string child_name = linkMap.find(link_name)->second.second;
        return child_name;
    };
    const std::string CrexLeg::linkToParentName(const std::shared_ptr<robotlib::Link> link) const
    {
        const std::string link_name = link->getName();
        const std::string parent_name = linkMap.find(link_name)->second.first;
        return parent_name;
    };
} // namespace aliengolib