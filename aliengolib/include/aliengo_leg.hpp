#ifndef _CREXLIB_CREX_LEG_HPP_
#define _CREXLIB_CREX_LEG_HPP_

#include <robotlib/leg.hpp>
#include <map>

namespace aliengolib
{
    const int NJOINTS = 3;
    const int NLINKS = 3;

    class CrexLeg : public robotlib::Leg<NJOINTS, NLINKS>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CrexLeg(const std::string &name,
                const std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> &joints,
                const std::array<std::shared_ptr<robotlib::Link>, NLINKS> &links);

        ~CrexLeg();

        virtual const std::string jointToChildName(const std::shared_ptr<robotlib::Joint> joint) const override;
        virtual const std::string jointToParentName(const std::shared_ptr<robotlib::Joint> joint) const override;
        virtual const std::string linkToChildName(const std::shared_ptr<robotlib::Link> link) const override;
        virtual const std::string linkToParentName(const std::shared_ptr<robotlib::Link> link) const override;

    private:
        const std::map<std::string, std::pair<std::string, std::string>> jointMap;
        const std::map<std::string, std::pair<std::string, std::string>> linkMap;
    };
} // namespace aliengolib

#endif // _CREXLIB_CREX_LEG_HPP_
