/*!
 * @file aliengo_leg.hpp
 *
 * @brief Aliengo leg class definition and functions prototypes.
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ALIENGOLIB_ALIENGO_LEG_HPP_
#define _ALIENGOLIB_ALIENGO_LEG_HPP_

#include <robotlib/leg.hpp>
#include <map>

namespace aliengolib
{
    //! Number of joints of the leg.
    const int NJOINTS = 3;
    //! Number of links of the leg.
    const int NLINKS = 3;

    /*!
    * @brief AliengoLeg class.
    * @details
    * This class creates the Aliengo leg.
    * @tparam NJOINTS number of joints of the leg.
    * @tparam NLINKS number of links of the leg.
    */
    class AliengoLeg : public robotlib::Leg<NJOINTS, NLINKS>
    {
    public:
        /*!
        * @brief Constructor.
        * @param[in] name name of the leg.
        * @param[in] joints array of shared pointers pointing to leg's joints.
        * @param[in] links array of shared pointers pointing to leg's links.
        */
        AliengoLeg(const std::string &name,
                const std::array<std::shared_ptr<robotlib::Joint>, NJOINTS> &joints,
                const std::array<std::shared_ptr<robotlib::Link>, NLINKS> &links);

        /*!
        * @brief Destructor.
        */
        ~AliengoLeg();

        /*!
        * @brief Get the name of the joint's parent of the leg.
        * @param[in] joint Joint object for which you get the name of the parent.
        * @return name of the joint's parent. 
        */
        virtual std::string jointToParentName(const std::shared_ptr<robotlib::Joint> joint) const override;

        /*!
        * @brief Get the name of the joint's child of the leg.
        * @param[in] joint Joint object for which you get the name of the child.
        * @return name of the joint's child.
        */
        virtual std::string jointToChildName(const std::shared_ptr<robotlib::Joint> joint) const override;

        /*!
        * @brief Get the name of link's parent of the leg.
        * @param[in] link Link object for which you get the name of the parent.
        * @return name of the link's parent.
        */
        virtual std::string linkToParentName(const std::shared_ptr<robotlib::Link> link) const override;

        /*!
        * @brief Get the name of the link's child of the leg.
        * @param[in] link Link object for which you get the name of the child.
        * @return name of the link's child.
        */
        virtual std::string linkToChildName(const std::shared_ptr<robotlib::Link> link) const override;

    private:
            //! Variable mapping each joint name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> joints_map_{};

            //! Variable mapping each link name to its parent and child names.
            std::map<std::string, std::pair<std::string, std::string>> links_map_{};
    };
} // namespace aliengolib

#endif // _ALIENGOLIB_ALIENGO_LEG_HPP_
