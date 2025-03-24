#ifndef _ALIENGOLIB_ALIENGO_LEG_HPP_
#define _ALIENGOLIB_ALIENGO_LEG_HPP_

#include <robotlib/leg.hpp>
#include <map>

namespace aliengolib
{
    //! Number of joints of the leg.
    const int NJOINTS_LEG = 3;
    //! Number of links of the leg.
    const int NLINKS_LEG = 3;

    /*!
    * @brief AliengoLeg class.
    * @details
    * This class creates the Aliengo leg.
    * @tparam NJOINTS_LEG number of joints of the leg.
    * @tparam NLINKS_LEG number of links of the leg.
    */
    class AliengoLeg : public robotlib::Leg<NJOINTS_LEG, NLINKS_LEG>
    {
    public:
        /*!
        * @brief Constructor.
        * @param[in] name name of the leg.
        * @param[in] joints array of shared pointers pointing to leg's joints.
        * @param[in] links array of shared pointers pointing to leg's links.
        */
        AliengoLeg(const std::string &name,
                const robotlib::Container<robotlib::Joint, NJOINTS_LEG> &joints,
                const robotlib::Container<robotlib::Link, NLINKS_LEG> &links);

        /*!
        * @brief Destructor.
        */
        ~AliengoLeg();

        /*!
        * @brief Get the name of the joint's parent of the leg.
        * @param[in] joint Joint object for which you get the name of the parent.
        * @return name of the joint's parent. 
        */
        virtual std::string jointToParentName(const std::shared_ptr<robotlib::Joint> joint) const;

        /*!
        * @brief Get the name of the joint's child of the leg.
        * @param[in] joint Joint object for which you get the name of the child.
        * @return name of the joint's child.
        */
        virtual std::string jointToChildName(const std::shared_ptr<robotlib::Joint> joint) const;

        /*!
        * @brief Get the name of link's parent of the leg.
        * @param[in] link Link object for which you get the name of the parent.
        * @return name of the link's parent.
        */
        virtual std::string linkToParentName(const std::shared_ptr<robotlib::Link> link) const;

        /*!
        * @brief Get the name of the link's child of the leg.
        * @param[in] link Link object for which you get the name of the child.
        * @return name of the link's child.
        */
        virtual std::string linkToChildName(const std::shared_ptr<robotlib::Link> link) const;

    private:
        //! Variable mapping each joint name to its parent and child names.
        const std::map<std::string, std::pair<std::string, std::string>> joints_map_{};

        //! Variable mapping each link name to its parent and child names.
        const std::map<std::string, std::pair<std::string, std::string>> links_map_{};
    };
} // namespace aliengolib

#endif // _ALIENGOLIB_ALIENGO_LEG_HPP_